#ifndef CUBICSPLINE_H
#define CUBICSPLINE_H
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>

template <typename T>
struct CubicSplineData
{
    T maxt;
    T n;
    T Ts;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Coef;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> t;
};

template <typename T>
struct KinematicsData
{
    T y;
    T dy;
    T ddy;
};

template <typename T>
struct SplineGeneratorData
{
    std::vector<T> tt;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cs_q;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cs_v;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cs_a;
};

template <typename T>
struct CroutData
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> L;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> U;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> v;
};

template <typename T>
class CubicSpline
{
public:
    bool CubicSplineDataCheck(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> q, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> t, T ts){
        if(q.cols() != t.cols() || ts <= 1e-6 || q.rows() != 4)
            return isGoodData = false;        
        return isGoodData = true;
    }

    bool GetDataCheck(){
        return isGoodData;
    }

    SplineGeneratorData<T> SplineGenerator(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> q, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> t, T ts)
    {
        KinematicsData<T> kinematicsData;
        CubicSplineData<T> cs;
        int m = q.rows();
        int n = q.cols();
        int totalnum = round(t(n - 1) / ts);
        t(n - 1) = totalnum * ts;
        std::vector<float> tt; //length 3001
        for (float i = 0; i < t(n - 1); i += ts)
            tt.push_back(i);

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cs_q = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(m, totalnum + 1);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cs_v = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(m, totalnum + 1);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cs_a = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(m, totalnum + 1);

        for (int i = 0; i < m; ++i)
        {
            cs = CubicSplineInterpolation(t, q.row(i), 0, 0, ts);

            for (int j = 0; j < totalnum + 1; ++j)
            {
                kinematicsData = GetCubicSpline(cs, ts * (j));
                cs_q(i, j) = kinematicsData.y;
                cs_v(i, j) = kinematicsData.dy;
                cs_a(i, j) = kinematicsData.ddy;
            }
        }

        SplineGeneratorData<T> splineGeneratorData;
        splineGeneratorData.tt = tt;
        splineGeneratorData.cs_q = cs_q;
        splineGeneratorData.cs_v = cs_v;
        splineGeneratorData.cs_a = cs_a;
        return splineGeneratorData;
    }

private:
    CubicSplineData<T> CubicSplineInterpolation(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> t, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> q, T vs, T ve, T Ts)
    {
        CubicSplineData<T> cs;
        int pointNum = t.cols();
        int n = pointNum - 1;
        int maxt = t(pointNum - 1);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_ = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n);
        for (int i = 0; i < n; ++i)
            T_(i) = t(i + 1) - t(i);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(n - 1, n - 1);
        for (int i = 0; i < n - 1; ++i)
        { //[0, n-2]
            A(i, i) = 2 * (T_(i) + T_(i + 1));
            if (i != n - 2)
                A(i, i + 1) = T_(i);
            if (i != 0)
                A(i, i - 1) = T_(i + 1);
        }
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> a = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n - 1);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> b = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n - 2);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(n - 1, 1);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> d = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n - 2);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Coef = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, 4);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> V = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n + 1);

        c(0) = 3 / (T_(0) * T_(1)) * ((T_(0) * T_(0) * (q(2) - q(1)) + T_(1) * T_(1) * (q(1) - q(0)))) - T_(1) * vs;
        c(n - 2) = 3 / (T_(n - 2) * T_(n - 1)) * ((T_(n - 2) * T_(n - 2) * (q(n) - q(n - 1)) + T_(n - 1) * T_(n - 1) * (q(n - 1) - q(n - 2)))) - T_(n - 2) * ve;
        for (int i = 0; i <= n - 4; ++i)
            c(i + 1) = 3 / (T_(1 + i) * T_(2 + i)) * ((T_(1 + i) * T_(1 + i) * (q(3 + i) - q(2 + i)) + T_(2 + i) * T_(2 + i) * (q(2 + i) - q(1 + i))));

        for (int i = 0; i < n - 1; ++i)
        { //[0, n-2]
            a(i) = A(i, i);
            if (i != n - 2)
            { //[0,n-4]
                b(i) = A(i, i + 1);
                d(i) = A(i + 1, i);
            }
        }
        CroutData<T> croutData = crout(a, b, d, c);

        V(0) = vs;
        V(n) = ve;
        for (int i = 1; i < n; ++i) //[1,n-1]
            V(i) = croutData.v(i - 1);

        for (int i = 0; i < n; ++i)
        {
            Coef(i, 0) = q(i);
            Coef(i, 1) = V(i);
            Coef(i, 2) = 1 / T_(i) * (3 * (q(i + 1) - q(i)) / T_(i) - 2 * V(i) - V(i + 1));
            Coef(i, 3) = 1 / (T_(i) * T_(i)) * (2 * (q(i) - q(i + 1)) / T_(i) + V(i) + V(i + 1));
        }
        cs.maxt = maxt;
        cs.n = n;
        cs.Coef = Coef;
        cs.Ts = Ts;
        cs.t = t;
        return cs;
    }

    CroutData<T> crout(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> a, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> c, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> d, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> b)
    {
        int n = a.size();
        // int n1 = c.size();
        // int n2 = d.size();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> L = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> U = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> q = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n - 1);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> y = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(1, n);
        CroutData<T> croutData;

        for (int i = 1; i <= n; ++i)
            p(i - 1) = i;
        for (int i = 1; i <= n - 1; ++i)
            q(i - 1) = i;
        for (int i = 1; i <= n; ++i)
            x(i - 1) = i;
        for (int i = 1; i <= n; ++i)
            y(i - 1) = i;
        p(0) = a(0);

        for (int i = 0; i < n - 1; ++i)
        {
            q(i) = c(i) / p(i);
            p(i + 1) = a(i + 1) - d(i) * q(i);
        }

        y(0) = b(0) / p(0);
        for (int i = 1; i < n; ++i) //[1, n-1]
            y(i) = (b(i) - d(i - 1) * y(i - 1)) / p(i);
        x(n - 1) = y(n - 1);
        for (int i = n - 1 - 1; i >= 0; --i)
            x(i) = y(i) - q(i) * x(i + 1);
        for (int i = 0; i < n; ++i)
        {
            L(i, i) = p(i);
            U(i, i) = 1;
        }
        for (int i = 0; i < n - 1; ++i)
        {
            L(i + 1, i) = d(i);
            U(i, i + 1) = q(i);
        }

        croutData.L = L;
        croutData.U = U;
        croutData.v = x;
        return croutData;
    }

    KinematicsData<T> GetCubicSpline(CubicSplineData<T> cs_, T t)
    {
        float actt = 0.0;
        float actPos = 0.0;
        float actVel = 0.0;
        float actAcc = 0.0;
        if (t - cs_.maxt > 1e-6)
            t = cs_.maxt;
        else if (t < 1e-6)
            t = 0;
        for (int k = 0; k < cs_.n; ++k)
        { //[0, cs.n-1]
            if (t - cs_.t(k) >= -1e-10 && t - cs_.t(k + 1) <= 1e-10)
            {
                actt = t - cs_.t(k);
                actPos = cs_.Coef(k, 0) + cs_.Coef(k, 1) * actt + cs_.Coef(k, 2) * actt * actt + cs_.Coef(k, 3) * actt * actt * actt;
                actVel = cs_.Coef(k, 1) + 2 * cs_.Coef(k, 2) * actt + 3 * cs_.Coef(k, 3) * actt * actt;
                actAcc = 2 * cs_.Coef(k, 2) + 6 * cs_.Coef(k, 3) * actt;
                break;
            }
        }
        KinematicsData<T> kinematicsData;
        kinematicsData.y = actPos;
        kinematicsData.dy = actVel;
        kinematicsData.ddy = actAcc;
        return kinematicsData;
    }

private:
    bool isGoodData = true;
};

#endif // CUBICSPLINE_H