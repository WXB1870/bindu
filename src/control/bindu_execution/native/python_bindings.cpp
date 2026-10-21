#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "interpolation.hpp"
#include <cstring>
#include <cstdint>
#include <stdexcept>

using namespace bindu::execution;
namespace {
struct Owned {
    PyObject* p;
    explicit Owned(PyObject* value):p(value) {}
    ~Owned(){Py_XDECREF(p);}
    Owned(const Owned&)=delete;
};
void check(bool ok,const char* message){if(!ok) throw std::invalid_argument(message);}
std::vector<double> vector(PyObject* value) {
    Owned seq(PySequence_Fast(value,"expected numeric sequence"));check(seq.p,"INVALID_SEQUENCE");
    const auto count=PySequence_Fast_GET_SIZE(seq.p);check(count<=4096,"SEQUENCE_TOO_LARGE");
    std::vector<double> result;result.reserve(count);
    for(Py_ssize_t i=0;i<count;++i) {
        const double x=PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq.p,i));
        check(!PyErr_Occurred(),"INVALID_NUMBER");result.push_back(x);
    }
    return result;
}
JointState state(PyObject* value) {
    Owned seq(PySequence_Fast(value,"expected q/v/a"));check(seq.p&&PySequence_Fast_GET_SIZE(seq.p)==3,"INVALID_STATE");
    JointState result{vector(PySequence_Fast_GET_ITEM(seq.p,0)),vector(PySequence_Fast_GET_ITEM(seq.p,1)),vector(PySequence_Fast_GET_ITEM(seq.p,2)),{}};
    result.j.assign(result.q.size(),0.);return result;
}
JointLimits limits(PyObject* value) {
    Owned seq(PySequence_Fast(value,"expected limits"));check(seq.p,"INVALID_LIMITS");
    const auto count=PySequence_Fast_GET_SIZE(seq.p);check(count>0&&count<=256,"INVALID_LIMITS");
    JointLimits out;
    for(Py_ssize_t i=0;i<count;++i) {
        auto row=vector(PySequence_Fast_GET_ITEM(seq.p,i));check(row.size()==5,"INVALID_LIMITS");
        Limits l;l.lower=row[0];l.upper=row[1];l.velocity=row[2];l.acceleration=row[3];l.jerk=row[4];out.push_back(l);
    }
    return out;
}
PyObject* tuple(const std::vector<double>& values) {
    PyObject* out=PyTuple_New(values.size());if(!out) return nullptr;
    for(size_t i=0;i<values.size();++i) {
        auto item=PyFloat_FromDouble(values[i]);if(!item){Py_DECREF(out);return nullptr;}
        PyTuple_SET_ITEM(out,i,item);
    }
    return out;
}
PyObject* state_tuple(const JointState& s) {
    Owned q(tuple(s.q)),v(tuple(s.v)),a(tuple(s.a)),j(tuple(s.j));
    if(!q.p||!v.p||!a.p||!j.p) return nullptr;
    return PyTuple_Pack(4,q.p,v.p,a.p,j.p);
}
constexpr const char* curve_name="bindu.execution.Curve";
constexpr const char* online_name="bindu.execution.Online";
constexpr const char* prepared_name="bindu.execution.PreparedSegment";
void free_curve(PyObject* obj){delete static_cast<std::shared_ptr<Curve>*>(PyCapsule_GetPointer(obj,curve_name));}
void free_online(PyObject* obj){delete static_cast<Online*>(PyCapsule_GetPointer(obj,online_name));}
void free_prepared(PyObject* obj){delete static_cast<PreparedSegment*>(PyCapsule_GetPointer(obj,prepared_name));}
PreparedSegment& prepared(PyObject* obj) {
    auto p=static_cast<PreparedSegment*>(PyCapsule_GetPointer(obj,prepared_name));
    check(p,"INVALID_PREPARED_HANDLE");return *p;
}
PyObject* prepared_capsule(PreparedSegment segment) {
    auto p=std::make_unique<PreparedSegment>(std::move(segment));
    auto obj=PyCapsule_New(p.get(),prepared_name,free_prepared);if(obj) p.release();return obj;
}
PyObject* capsule(std::shared_ptr<Curve> c) {
    auto p=new std::shared_ptr<Curve>(std::move(c));auto out=PyCapsule_New(p,curve_name,free_curve);
    if(!out) delete p;return out;
}
Curve& curve(PyObject* obj) {
    auto p=static_cast<std::shared_ptr<Curve>*>(PyCapsule_GetPointer(obj,curve_name));check(p,"INVALID_CURVE_HANDLE");return **p;
}
PyObject* capsules(const Curves& curves) {
    auto out=PyTuple_New(curves.size());if(!out) return nullptr;
    for(size_t i=0;i<curves.size();++i) {
        auto c=capsule(curves[i]);if(!c){Py_DECREF(out);return nullptr;}PyTuple_SET_ITEM(out,i,c);
    }
    return out;
}
PyObject* error() {
    try {throw;}
    catch(const std::bad_alloc&){PyErr_NoMemory();}
    catch(const std::exception& e){PyErr_SetString(PyExc_ValueError,e.what());}
    return nullptr;
}
PyObject* between(PyObject*,PyObject* args) {
    double start,duration;PyObject *initial,*final;
    if(!PyArg_ParseTuple(args,"ddOO",&start,&duration,&initial,&final)) return nullptr;
    try {check(duration>=1e-5,"TRAJECTORY_INTERVAL_TOO_SHORT");return capsule(std::make_shared<Curve>(start,duration,state(initial),state(final)));}catch(...){return error();}
}
PyObject* info(PyObject*,PyObject* args) {
    PyObject* obj;if(!PyArg_ParseTuple(args,"O",&obj)) return nullptr;
    try {auto& c=curve(obj);return Py_BuildValue("dd",c.start,c.duration);}catch(...){return error();}
}
PyObject* curve_sample(PyObject*,PyObject* args) {
    PyObject* obj;double now;if(!PyArg_ParseTuple(args,"Od",&obj,&now)) return nullptr;
    try {return state_tuple(curve(obj).sample(now));}catch(...){return error();}
}
PyObject* curve_valid(PyObject*,PyObject* args) {
    PyObject *obj,*ls;if(!PyArg_ParseTuple(args,"OO",&obj,&ls)) return nullptr;
    try {return PyBool_FromLong(curve(obj).valid(limits(ls)));}catch(...){return error();}
}
PyObject* target(PyObject*,PyObject* args) {
    PyObject *ls,*s,*goal;double start,horizon,timeout;
    if(!PyArg_ParseTuple(args,"OOOddd",&ls,&s,&goal,&start,&horizon,&timeout)) return nullptr;
    try {return capsules(fit_target(limits(ls),state(s),vector(goal),start,horizon,timeout));}catch(...){return error();}
}
PyObject* stop(PyObject*,PyObject* args) {
    PyObject *ls,*s;double start,timeout;
    if(!PyArg_ParseTuple(args,"OOdd",&ls,&s,&start,&timeout)) return nullptr;
    try {return capsules(fit_stop(limits(ls),state(s),start,timeout));}catch(...){return error();}
}
PyObject* timed(PyObject*,PyObject* args) {
    PyObject *ls,*s,*ps,*ts;double start,timeout;int chunk;
    if(!PyArg_ParseTuple(args,"OOOOdpd",&ls,&s,&ps,&ts,&start,&chunk,&timeout)) return nullptr;
    try {
        Owned seq(PySequence_Fast(ps,"expected points"));check(seq.p,"INVALID_POINTS");
        const auto count=PySequence_Fast_GET_SIZE(seq.p);check(count>0&&count<=1024,"TRAJECTORY_TOO_LARGE");
        std::vector<JointState> points;for(Py_ssize_t i=0;i<count;++i) points.push_back(state(PySequence_Fast_GET_ITEM(seq.p,i)));
        return capsules(fit_timed(limits(ls),state(s),points,vector(ts),start,chunk,timeout));
    }catch(...){return error();}
}
PyObject* online(PyObject*,PyObject* args) {
    PyObject *ls,*s,*goal;double start,period,horizon;
    if(!PyArg_ParseTuple(args,"OOOddd",&ls,&s,&goal,&start,&period,&horizon)) return nullptr;
    try {
        auto p=std::make_unique<Online>(limits(ls),state(s),vector(goal),start,period,horizon);
        auto obj=PyCapsule_New(p.get(),online_name,free_online);if(obj) p.release();return obj;
    }catch(...){return error();}
}
PyObject* online_sample(PyObject*,PyObject* args) {
    PyObject* obj;double now;if(!PyArg_ParseTuple(args,"Od",&obj,&now)) return nullptr;
    try {
        auto p=static_cast<Online*>(PyCapsule_GetPointer(obj,online_name));check(p,"INVALID_ONLINE_HANDLE");
        return state_tuple(p->sample(now));
    }catch(...){return error();}
}
PyObject* prepared_target(PyObject*,PyObject* args) {
    PyObject *ls,*s,*goal;double start,horizon,timeout;
    if(!PyArg_ParseTuple(args,"OOOddd",&ls,&s,&goal,&start,&horizon,&timeout)) return nullptr;
    try {return prepared_capsule(PreparedSegment::target(limits(ls),state(s),vector(goal),start,horizon,timeout));}
    catch(...){return error();}
}
PyObject* prepared_stop(PyObject*,PyObject* args) {
    PyObject *ls,*s;double start,timeout;
    if(!PyArg_ParseTuple(args,"OOdd",&ls,&s,&start,&timeout)) return nullptr;
    try {return prepared_capsule(PreparedSegment::stop(limits(ls),state(s),start,timeout));}
    catch(...){return error();}
}
PyObject* prepared_info(PyObject*,PyObject* args) {
    PyObject* obj;if(!PyArg_ParseTuple(args,"O",&obj)) return nullptr;
    try {const auto& p=prepared(obj);return Py_BuildValue("ddnn",p.start(),p.end(),
        static_cast<Py_ssize_t>(p.axes()),static_cast<Py_ssize_t>(p.pieces()));}catch(...){return error();}
}
PyObject* prepared_sample(PyObject*,PyObject* args) {
    PyObject* obj;double now;if(!PyArg_ParseTuple(args,"Od",&obj,&now)) return nullptr;
    try {return state_tuple(prepared(obj).sample(now));}catch(...){return error();}
}
PyObject* prepared_describe(PyObject*,PyObject* args) {
    PyObject* obj;if(!PyArg_ParseTuple(args,"O",&obj)) return nullptr;
    try {
        const auto description=prepared(obj).describe();
        Owned out(PyTuple_New(description.size()));if(!out.p) return nullptr;
        for(size_t k=0;k<description.size();++k) {
            const auto& d=description[k];Owned controls(PyTuple_New(d.controls.size()));
            if(!controls.p) return nullptr;
            for(size_t axis=0;axis<d.controls.size();++axis) {
                auto p=tuple(d.controls[axis]);if(!p) return nullptr;PyTuple_SET_ITEM(controls.p,axis,p);
            }
            auto piece=Py_BuildValue("ddddO",d.start,d.duration,d.source_duration,d.source_elapsed,controls.p);
            if(!piece) return nullptr;PyTuple_SET_ITEM(out.p,k,piece);
        }
        Py_INCREF(out.p);return out.p;
    }catch(...){return error();}
}
PyObject* prepared_from_description(PyObject*,PyObject* args) {
    PyObject *ls,*value;if(!PyArg_ParseTuple(args,"OO",&ls,&value)) return nullptr;
    try {
        Owned seq(PySequence_Fast(value,"expected prepared pieces"));check(seq.p,"INVALID_PREPARED_LAYOUT");
        const auto count=PySequence_Fast_GET_SIZE(seq.p);
        check(count>0&&count<=static_cast<Py_ssize_t>(PreparedSegment::max_pieces),"INVALID_PREPARED_LAYOUT");
        std::vector<CurveDescriptor> pieces;pieces.reserve(count);
        for(Py_ssize_t k=0;k<count;++k) {
            Owned row(PySequence_Fast(PySequence_Fast_GET_ITEM(seq.p,k),"expected curve descriptor"));
            check(row.p&&PySequence_Fast_GET_SIZE(row.p)==5,"INVALID_CURVE_DESCRIPTION");
            auto at=[&](Py_ssize_t index){return PySequence_Fast_GET_ITEM(row.p,index);};
            CurveDescriptor d;d.start=PyFloat_AsDouble(at(0));d.duration=PyFloat_AsDouble(at(1));
            d.source_duration=PyFloat_AsDouble(at(2));d.source_elapsed=PyFloat_AsDouble(at(3));
            check(!PyErr_Occurred(),"INVALID_CURVE_DESCRIPTION_TIME");
            Owned axes(PySequence_Fast(at(4),"expected axis controls"));check(axes.p,"INVALID_CURVE_DESCRIPTION_AXES");
            const auto n=PySequence_Fast_GET_SIZE(axes.p);
            check(n>0&&n<=static_cast<Py_ssize_t>(PreparedSegment::max_axes),"INVALID_CURVE_DESCRIPTION_AXES");
            for(Py_ssize_t axis=0;axis<n;++axis) d.controls.push_back(vector(PySequence_Fast_GET_ITEM(axes.p,axis)));
            pieces.push_back(std::move(d));
        }
        return prepared_capsule(PreparedSegment::from_description(pieces,limits(ls)));
    }catch(...){return error();}
}
struct Buffer {
    Py_buffer view{};
    ~Buffer(){if(view.obj) PyBuffer_Release(&view);}
    double* get(PyObject* obj,size_t n) {
        check(PyObject_GetBuffer(obj,&view,PyBUF_WRITABLE|PyBUF_FORMAT|PyBUF_C_CONTIGUOUS)==0,"INVALID_OUTPUT_BUFFER");
        check(view.itemsize==sizeof(double)&&view.format&&std::strcmp(view.format,"d")==0&&
              view.len>=static_cast<Py_ssize_t>(n*sizeof(double))&&
              reinterpret_cast<uintptr_t>(view.buf)%alignof(double)==0,"INVALID_OUTPUT_BUFFER");
        return static_cast<double*>(view.buf);
    }
};
PyObject* prepared_sample_into(PyObject*,PyObject* args) {
    PyObject *obj,*q,*v,*a,*j;double now;
    if(!PyArg_ParseTuple(args,"OdOOOO",&obj,&now,&q,&v,&a,&j)) return nullptr;
    try {
        const auto& p=prepared(obj);const auto n=p.axes();Buffer buffers[4];
        double* out[]={buffers[0].get(q,n),buffers[1].get(v,n),buffers[2].get(a,n),buffers[3].get(j,n)};
        for(size_t x=0;x<4;++x) for(size_t y=0;y<x;++y) {
            const auto lhs=reinterpret_cast<uintptr_t>(out[x]),rhs=reinterpret_cast<uintptr_t>(out[y]);
            check(lhs>=rhs+n*sizeof(double)||rhs>=lhs+n*sizeof(double),"OVERLAPPING_OUTPUT_BUFFERS");
        }
        check(p.sample_into(now,out[0],out[1],out[2],out[3],n),"INVALID_PREPARED_SAMPLE");
        Py_RETURN_NONE;
    }catch(...){return error();}
}
PyMethodDef methods[]={
    {"between",between,METH_VARARGS,nullptr},{"curve_info",info,METH_VARARGS,nullptr},
    {"curve_sample",curve_sample,METH_VARARGS,nullptr},{"curve_valid",curve_valid,METH_VARARGS,nullptr},
    {"fit_target",target,METH_VARARGS,nullptr},{"fit_stop",stop,METH_VARARGS,nullptr},
    {"fit_timed",timed,METH_VARARGS,nullptr},{"online",online,METH_VARARGS,nullptr},
    {"online_sample",online_sample,METH_VARARGS,nullptr},
    {"prepared_target",prepared_target,METH_VARARGS,nullptr},{"prepared_stop",prepared_stop,METH_VARARGS,nullptr},
    {"prepared_info",prepared_info,METH_VARARGS,nullptr},{"prepared_sample",prepared_sample,METH_VARARGS,nullptr},
    {"prepared_describe",prepared_describe,METH_VARARGS,nullptr},
    {"prepared_from_description",prepared_from_description,METH_VARARGS,nullptr},
    {"prepared_sample_into",prepared_sample_into,METH_VARARGS,nullptr},{nullptr,nullptr,0,nullptr}};
PyModuleDef module={PyModuleDef_HEAD_INIT,"_native","Shared C++ reference engine",-1,methods,nullptr,nullptr,nullptr,nullptr};
}
PyMODINIT_FUNC PyInit__native(){return PyModule_Create(&module);}
