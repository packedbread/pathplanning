#pragma once
#include <cmath>
#include <ostream>


namespace planner {
    template <typename T>
    struct Sqrt2 {
        static constexpr T value = static_cast<T>(1.4142135623730950488016887242096980785696718753769480731766797);
    };

    // todo: consider removing `Root` parameter, as it'll be unused until multiplication operation is introduced
    template <typename T, typename Root = Sqrt2<double>>
    struct Quadratic {
        T real;
        T imaginary;

        Quadratic() : real(), imaginary() {}
        Quadratic(T real, T imaginary = {}) : real(real), imaginary(imaginary) {}

        Quadratic& operator += (Quadratic other) {
            real += other.real;
            imaginary += other.imaginary;
            return *this;
        }

        Quadratic& operator -= (Quadratic other) {
            real -= other.real;
            imaginary -= other.imaginary;
            return *this;
        }

        bool operator == (const Quadratic& other) const {
            return real == other.real && imaginary == other.imaginary;
        }
    };

    template <typename T, typename Root, typename U>
    Quadratic<T, Root> operator + (Quadratic<T, Root> a, U b) {
        Quadratic<T, Root> result = a;
        result += b;
        return result;
    }

    template <typename T, typename Root, typename U>
    Quadratic<T, Root> operator - (Quadratic<T, Root> a, U b) {
        Quadratic<T, Root> result = a;
        result -= b;
        return result;
    }

    template <typename T, typename Root>
    std::ostream& operator << (std::ostream& out, const Quadratic<T, Root>& q) {
        return out << '(' << q.real << ", " << q.imaginary << ')';
    }

    using number = Quadratic<int, Sqrt2<double>>;

    template <typename T, template <typename> class Root>
    double evaluate(Quadratic<T, Root<double>> quadratic) {
        return quadratic.real + quadratic.imaginary * Root<double>::value;
    }
}
