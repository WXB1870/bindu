#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "interpolation.hpp"
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
void free_curve(PyObject* obj){delete static_cast<std::shared_ptr<Curve>*>(PyCapsule_GetPointer(obj,curve_name));}
void free_online(PyObject* obj){delete static_cast<Online*>(PyCapsule_GetPointer(obj,online_name));}
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
PyMethodDef methods[]={
    {"between",between,METH_VARARGS,nullptr},{"curve_info",info,METH_VARARGS,nullptr},
    {"curve_sample",curve_sample,METH_VARARGS,nullptr},{"curve_valid",curve_valid,METH_VARARGS,nullptr},
    {"fit_target",target,METH_VARARGS,nullptr},{"fit_stop",stop,METH_VARARGS,nullptr},
    {"fit_timed",timed,METH_VARARGS,nullptr},{"online",online,METH_VARARGS,nullptr},
    {"online_sample",online_sample,METH_VARARGS,nullptr},{nullptr,nullptr,0,nullptr}};
PyModuleDef module={PyModuleDef_HEAD_INIT,"_native","Shared C++ reference engine",-1,methods,nullptr,nullptr,nullptr,nullptr};
}
PyMODINIT_FUNC PyInit__native(){return PyModule_Create(&module);}
