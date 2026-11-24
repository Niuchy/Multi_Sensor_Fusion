#include <cstring>
#include <cmath>
#if !defined(MATRIX3_H_INCLUDED_)
#define MATRIX3_H_INCLUDED_
class CMatrix3
{
public:
    int m; // 行数
    int n; // 列数
    union
    {
        double array[9];
        double a[3][3];
    };

public:
    CMatrix3();
    CMatrix3(CMatrix3 &other_matrix);
    CMatrix3(int i, int j = 1);

    CMatrix3(int i, int j, double *p);

    void SetRowCol(int i, int j);

    double get(int &i, int &j) { return array[(i - 1) * n + j - 1]; };
    void set(int i, int j, double k) { array[(i - 1) * n + j - 1] = k; };

    CMatrix3 operator=(const double *p);
    CMatrix3 operator=(const CMatrix3 &other_matrix);
    CMatrix3 operator=(const double k);

    CMatrix3 operator+(const CMatrix3 other_matrix);
    CMatrix3 operator-(const CMatrix3 other_matrix);
    CMatrix3 operator*(CMatrix3 other_matrix);
    CMatrix3 operator*(double k);

    CMatrix3 del_elem(int &, int &); //--

    CMatrix3 t();   //--求转置
    CMatrix3 rev(); //--求逆

    friend double operator+(double k, CMatrix3 x);
    friend double operator+(CMatrix3 x, double k);
    friend double operator-(double k, CMatrix3 x);
    friend double operator-(CMatrix3 x, double k);
    friend CMatrix3 operator*(double k, CMatrix3 x);
};

extern int matrix_error;

#endif //! defined(MATRIX3_H_INCLUDED_)
