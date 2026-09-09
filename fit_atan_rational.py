"""Fit x*(A+B*x*x)/(1+C*x*x) to atan(x) on [0,1] by Remez exchange.

Extrema are located on a dense grid, so the returned error is sampled rather
than a certified continuous-domain bound. No third-party dependencies.
"""
import math


def solve(matrix, rhs):
    """Small dense linear solve with partial pivoting."""
    rows = [row[:] + [value] for row, value in zip(matrix, rhs)]
    for i in range(len(rhs)):
        pivot = max(range(i, len(rhs)), key=lambda k: abs(rows[k][i]))
        rows[i], rows[pivot] = rows[pivot], rows[i]
        value = rows[i][i]
        rows[i] = [x / value for x in rows[i]]
        for j in range(len(rhs)):
            if j != i:
                value = rows[j][i]
                rows[j] = [x - value*y for x, y in zip(rows[j], rows[i])]
    return [row[-1] for row in rows]


def fit():
    params = [1.0, 0.26, 0.6, 1e-4]  # A, B, C, signed equal-ripple error
    extrema = [0.15, 0.5, 0.8, 1.0]
    for _ in range(20):
        # Newton solve of four alternating-error equations. Multiplication
        # by the rational denominator introduces the C*error cross term.
        for _ in range(20):
            a, b, c, error = params
            matrix, rhs = [], []
            for i, x in enumerate(extrema):
                f = math.atan(x)
                sign = (-1)**i
                matrix.append([x, x**3, -(f+sign*error)*x*x,
                               -sign*(1+c*x*x)])
                rhs.append(-(a*x+b*x**3-(f+sign*error)*(1+c*x*x)))
            delta = solve(matrix, rhs)
            params = [x+y for x, y in zip(params, delta)]
            if max(map(abs, delta)) < 1e-14:
                break
        a, b, c, error = params
        grid = []
        for i in range(20001):
            x = i / 20000
            grid.append((a*x+b*x**3)/(1+c*x*x)-math.atan(x))
        updated = [i/20000 for i in range(1, 20000)
                   if (grid[i]-grid[i-1])*(grid[i+1]-grid[i]) < 0] + [1.0]
        if len(updated) != 4:
            raise RuntimeError((params, updated))
        if extrema == updated:
            break
        extrema = updated
    return params, extrema, max(map(abs, grid))


if __name__ == "__main__":
    print(fit())
