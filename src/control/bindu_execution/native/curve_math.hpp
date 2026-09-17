#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
namespace bindu::execution::detail {
inline std::vector<double> deriv(const std::vector<double>& p, double duration) {
    std::vector<double> out;
    for (size_t i=1; i<p.size(); ++i) out.push_back((p.size()-1)*(p[i]-p[i-1])/duration);
    return out;
}
inline double eval(std::vector<double> p, double u) {
    while (p.size()>1) {
        for (size_t i=1; i<p.size(); ++i) p[i-1]=(1-u)*p[i-1]+u*p[i];
        p.pop_back();
    }
    return p.front();
}
inline bool bounded(std::vector<double> p, double lo, double hi, int depth=9) {
    constexpr double eps=1e-9;
    if (!std::all_of(p.begin(),p.end(),[](double x){return std::isfinite(x);})) return false;
    if (*std::min_element(p.begin(),p.end())>=lo-eps&&*std::max_element(p.begin(),p.end())<=hi+eps) return true;
    if (!depth||p.front()<lo-eps||p.front()>hi+eps||p.back()<lo-eps||p.back()>hi+eps) return false;
    std::vector<double> left{p.front()}, right{p.back()};
    while (p.size()>1) {
        for (size_t i=1; i<p.size(); ++i) p[i-1]=(p[i-1]+p[i])/2;
        p.pop_back(); left.push_back(p.front()); right.push_back(p.back());
    }
    std::reverse(right.begin(),right.end());
    return bounded(left,lo,hi,depth-1)&&bounded(right,lo,hi,depth-1);
}
}
