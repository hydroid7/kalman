module KalmanFilter

using LinearAlgebra

export Kalman, fusion, predict, K, next

const Matrix = AbstractArray{T, N} where {N, T <: Real}
const Vector = AbstractArray{T, N} where {N, T <: Real}

"""
Represents the Kalman-Filter object.

# Arguments
- `A`: State transition matrix.
- `Q`: Process covariance. Belongs to the state transition matrix.
- `G`: Measurement mapping matrix. It maps the measurement `y` in the domain of the state `x̂`.
- `R`: Measurement covariance matrix. It shows the uncertianty of the measurement. It belongs to `G`.
- `x̂`: State vector. Holds the current state. It should be initialized with a good approximation of the initial state.
- `Σ`: Uncertianty of the state vector. It should also be initialized with a good approximation of the uncertianty of the initial state.
- `B`: Control signal matrix. It maps the control signal `u` in the domain of the state `x̂`.

# Examples

Here is a simple model for two dimensional movement:
```
julia> x_0 = zeros(6, 1)
6×1 Array{Float64,2}:
 0.0
 0.0
 0.0
 0.0
 0.0
 0.0

julia> Σ_0 = Matrix{Float64}(I, 6, 6) .* 1000
6×6 Array{Float64,2}:
 1000.0     0.0     0.0     0.0     0.0     0.0
    0.0  1000.0     0.0     0.0     0.0     0.0
    0.0     0.0  1000.0     0.0     0.0     0.0
    0.0     0.0     0.0  1000.0     0.0     0.0
    0.0     0.0     0.0     0.0  1000.0     0.0
    0.0     0.0     0.0     0.0     0.0  1000.0

julia> Δ_t = 0.1
0.1

julia> A = [1 0 Δ_t 0 0.5*Δ_t^2 0;
            0 1 0 Δ_t 0 0.5*Δ_t^2;
            0 0 1 0 Δ_t 0;
            0 0 0 1 0 Δ_t;
            0 0 0 0 1 0;
            0 0 0 0 0 1]
6×6 Array{Float64,2}:
 1.0  0.0  0.1  0.0  0.005  0.0
 0.0  1.0  0.0  0.1  0.0    0.005
 0.0  0.0  1.0  0.0  0.1    0.0
 0.0  0.0  0.0  1.0  0.0    0.1
 0.0  0.0  0.0  0.0  1.0    0.0
 0.0  0.0  0.0  0.0  0.0    1.0

julia> G = [1 0 0 0 0 0;
            0 1 0 0 0 0;
            0 0 0 0 1 0;
            0 0 0 0 0 1]
4×6 Array{Int64,2}:
 1  0  0  0  0  0
 0  1  0  0  0  0
 0  0  0  0  1  0
 0  0  0  0  0  1

julia> R = [2 0 0 0;
            0 10 0 0;
            0 0 0.4 0;
            0 0 0 0.4]
4×4 Array{Float64,2}:
 2.0   0.0  0.0  0.0
 0.0  10.0  0.0  0.0
 0.0   0.0  0.4  0.0
 0.0   0.0  0.0  0.4

julia> Q = Matrix{Float64}(I, 6, 6) * 0.0001
6×6 Array{Float64,2}:
 0.0001  0.0     0.0     0.0     0.0     0.0
 0.0     0.0001  0.0     0.0     0.0     0.0
 0.0     0.0     0.0001  0.0     0.0     0.0
 0.0     0.0     0.0     0.0001  0.0     0.0
 0.0     0.0     0.0     0.0     0.0001  0.0
 0.0     0.0     0.0     0.0     0.0     0.0001

julia> Kalman(A, Q, G, R, x_0, Σ_0)
Kalman([1.0 0.0 … 0.005 0.0; 0.0 1.0 … 0.0 0.005; … ; 0.0 0.0 … 1.0 0.0; 0.0 0.0 … 0.0 1.0], [0.0001 0.0 … 0.0 0.0; 0.0 0.0001 … 0.0 0.0; … ; 0.0 0.0 … 0.0001 0.0; 0.0 0.0 … 0.0 0.0001], [1 0 … 0 0; 0 1 … 0 0; 0 0 … 1 0; 0 0 … 0 1], [2.0 0.0 0.0 0.0; 0.0 10.0 0.0 0.0; 0.0 0.0 0.4 0.0; 0.0 0.0 0.0 0.4], [0.0; 0.0; … ; 0.0; 0.0], [1000.0 0.0 … 0.0 0.0; 0.0 1000.0 … 0.0 0.0; … ; 0.0 0.0 … 1000.0 0.0; 0.0 0.0 … 0.0 1000.0], UniformScaling{Bool}true*I)
```
Other model with scalar values:
```
julia> A = 1
1

julia> Q = 0.9
0.9

julia> G = 1
1

julia> R = 0.8
0.8

julia> x_0 = 8
8

julia> Σ_0 = 1
1
julia> Kalman(A, Q, G, R, x_0, Σ_0)
Kalman(1, 0.9, 1, 0.8, 8, 1, 0)
```


See also: [`dimension_helper`](@ref), For the multi dimensional case: [`Kalman(A, Q, G, R, x̂, Σ, B)`](@ref), [`Kalman(A, Q, G, R, x̂, Σ)`](@ref) and [`Kalman(A, Q, G, R, x̂, Σ[, B = 0])`](@ref) for the scalar case.
"""
mutable struct Kalman
    A # State transition
    Q # Process covariance matrix
    G # Measurement mapping
    R # Measurement covariance matrix
    x̂ # Current estimate mean
    Σ # Current estimate uncertianty
    B # Control signal mapping matrix

    function Kalman(A::Matrix, Q::Matrix, G::Matrix, R::Matrix, x̂::Vector, Σ::Matrix, B::Matrix)
        size(A)[1] == size(A)[2] || throw(DimensionMismatch("State transition matrix A should be n x n. Given $(size(A))"))
        size(x̂)[1] == size(A)[1] && size(x̂)[2] == 1 || throw(DimensionMismatch("State Vector x̂ shoud be ($(size(A)[1]), 1). Given $(size(x̂))"))
        size(Σ) == size(A) || throw(DimensionMismatch("State covariance matrix Σ should be n x n. Given $(size(Σ))"))

        size(B)[1] == size(A)[1] || throw(DimensionMismatch("Control matrix B should be n x a. Given $(size(B))"))

        size(G)[2] == size(A)[1] || throw(DimensionMismatch("Measurement matrix G should be (b, $(size(A)[1])). Given $(size(G))"))

        size(Q) == size(A) || throw(DimensionMismatch("Process noise covariance matrix should be n x n. Given $(size(Q))"))
        size(R)[1] == size(R)[2] ||throw(DimensionMismatch("Measurement noise covariance matrix should square. Given $(size(R))"))
        size(R)[1] == size(G)[1] || throw(DimensionMismatch("Measurement noise covariance matrix should $(size(G)[1]) x $(size(G)[1]). Given $(size(R))"))

        new(A, Q, G, R, x̂, Σ, B)
    end

    """
    Creates a Kalman-Filter object without control signal mapping matrix.

    """
    Kalman(A::Matrix, Q::Matrix, G::Matrix, R::Matrix, x̂::Vector, Σ::Matrix) = Kalman(A, Q, G, R, x̂, Σ, zeros(size(A)))

    """
    Creates a Kalman-Filter object with optional control signal mapping matrix `B` for the scalar case without dimension checking.

    See also: [`Kalman`](@ref) for other constructors.
    """
    Kalman(A::Real, Q::Real, G::Real, R::Real, x̂::Real, Σ::Real, B::Real = 0) = new(A, Q, G, R, x̂, Σ, B)
end

"""
    dimension_helper(Kalman)

Helps to determine the dimension of the measurement vector `y` and the control signal `u` of the `Kalman` model.

# Examples
```
julia> dimension_helper(model)
(measurement = (6, 1), control_signal = (6, 1))
```

See also [`Kalman`](@ref)
"""
function dimension_helper(k::Kalman)
    if isa(k.A, Matrix)
        ( measurement = (size(k.G)[2], 1), control_signal = (size(k.B)[2], 1) )
    else
        @warn "Your model contains only scalars. `y` and `u` should be also scalars."
        ( measurement = (1, 1), control_signal = (1, 1) )
    end
end

"""
    K(Kalman)

Computes the Kalman Gain based on a model.

# Arguments
The only argument is a valid `Kalman` struct.

# Examples
```
julia> model = Kalman(A, Q, G, R, x_0, Σ_0, B)
Kalman([1.0 0.0 … 0.005 0.0; 0.0 1.0 … 0.0 0.005; … ; 0.0 0.0 … 1.0 0.0; 0.0 0.0 … 0.0 1.0], [0.0001 0.0 … 0.0 0.0; 0.0 0.0001 … 0.0 0.0; … ; 0.0 0.0 … 0.0001 0.0; 0.0 0.0 … 0.0 0.0001], [1 0 … 0 0; 0 1 … 0 0; 0 0 … 1 0; 0 0 … 0 1], [2.0 0.0 0.0 0.0; 0.0 10.0 0.0 0.0; 0.0 0.0 0.4 0.0; 0.0 0.0 0.0 0.4], [0.0; 0.0; … ; 0.0; 0.0], [1000.0 0.0 … 0.0 0.0; 0.0 1000.0 … 0.0 0.0; … ; 0.0 0.0 … 1000.0 0.0; 0.0 0.0 … 0.0 1000.0], [1.0 0.0 … 0.0 0.0; 0.0 1.0 … 0.0 0.0; … ; 0.0 0.0 … 1.0 0.0; 0.0 0.0 … 0.0 1.0])
julia> K(model)
6×4 Array{Float64,2}:
 0.998004  0.0       0.0     0.0
 0.0       0.990099  0.0     0.0
 0.0       0.0       0.0     0.0
 0.0       0.0       0.0     0.0
 0.0       0.0       0.9996  0.0
 0.0       0.0       0.0     0.9996
```
See also: [`Kalman`](@ref)

"""
function K(k::Kalman)
    K(k.Σ, k.G, k.R)
end

"""
    K(Σ, G, R)

Computes the Kalman Gain based the matrices.
# Arguments
- Σ: Current state covariance
- G: Measurement matrix
- R: Measurement covariance matrix

# Examples
```
julia> model = Kalman(A, Q, G, R, x_0, Σ_0, B)
Kalman([1.0 0.0 … 0.005 0.0; 0.0 1.0 … 0.0 0.005; … ; 0.0 0.0 … 1.0 0.0; 0.0 0.0 … 0.0 1.0], [0.0001 0.0 … 0.0 0.0; 0.0 0.0001 … 0.0 0.0; … ; 0.0 0.0 … 0.0001 0.0; 0.0 0.0 … 0.0 0.0001], [1 0 … 0 0; 0 1 … 0 0; 0 0 … 1 0; 0 0 … 0 1], [2.0 0.0 0.0 0.0; 0.0 10.0 0.0 0.0; 0.0 0.0 0.4 0.0; 0.0 0.0 0.0 0.4], [0.0; 0.0; … ; 0.0; 0.0], [1000.0 0.0 … 0.0 0.0; 0.0 1000.0 … 0.0 0.0; … ; 0.0 0.0 … 1000.0 0.0; 0.0 0.0 … 0.0 1000.0], [1.0 0.0 … 0.0 0.0; 0.0 1.0 … 0.0 0.0; … ; 0.0 0.0 … 1.0 0.0; 0.0 0.0 … 0.0 1.0])
julia> K(model.Σ, model.G, model.R)
6×4 Array{Float64,2}:
 0.998004  0.0       0.0     0.0
 0.0       0.990099  0.0     0.0
 0.0       0.0       0.0     0.0
 0.0       0.0       0.0     0.0
 0.0       0.0       0.9996  0.0
 0.0       0.0       0.0     0.9996
```

See also: [`Kalman`](@ref)
"""
function K(Σ, G, R)
    Σ * transpose(G) * (G * Σ * transpose(G) + R)^-1
end

"""
    next(k::Kalman, y [, u])

Compute a complete Kalman Step. The dimension of the measurement vector `y` and control signal `u` must match with [`dimension_helper`](@ref)

# Examples
```
julia> model = Kalman(A, Q, G, R, x_0, Σ_0, B)
Kalman([1.0 0.0 … 0.005 0.0; 0.0 1.0 … 0.0 0.005; … ; 0.0 0.0 … 1.0 0.0; 0.0 0.0 … 0.0 1.0], [0.0001 0.0 … 0.0 0.0; 0.0 0.0001 … 0.0 0.0; … ; 0.0 0.0 … 0.0001 0.0; 0.0 0.0 … 0.0 0.0001], [1 0 … 0 0; 0 1 … 0 0; 0 0 … 1 0; 0 0 … 0 1], [2.0 0.0 0.0 0.0; 0.0 10.0 0.0 0.0; 0.0 0.0 0.4 0.0; 0.0 0.0 0.0 0.4], [0.0; 0.0; … ; 0.0; 0.0], [1000.0 0.0 … 0.0 0.0; 0.0 1000.0 … 0.0 0.0; … ; 0.0 0.0 … 1000.0 0.0; 0.0 0.0 … 0.0 1000.0], [1.0 0.0 … 0.0 0.0; 0.0 1.0 … 0.0 0.0; … ; 0.0 0.0 … 1.0 0.0; 0.0 0.0 … 0.0 1.0])
julia> next(model, [1 1 1 1 1 1]')
```

See also: [`dimension_helper`](@ref)
"""
function next(k::Kalman, y, u)
    newInstance = deepcopy(k)
    p = predict(newInstance, u).state
    f = fusion(newInstance, y)
    (model = newInstance, fusioned = f.state, predicted = p, gain = f.gain)
end
function next(k::Kalman, y)
    newInstance = deepcopy(k)
    p = predict(newInstance).state
    f = fusion(newInstance, y)
    (model = newInstance, fusioned = f.state, predicted = p, gain = f.gain)
end

"""
    fusion(k::Kalman, y)

Compute the filtered distribution.
"""
function fusion(k::Kalman, y)
    if size(y) != dimension_helper(newInstance).measurement
        @warn "The dimension of the measurement or the measurement matrix doesn't match."
    end
    g = K(k.Σ, k.G, k.R)
    k.x̂ = k.x̂ + g * (y - k.G * k.x̂)
    k.Σ = (I - g * k.G) * k.Σ * transpose(I - g * k.G) + g * k.R * transpose(g)
    (state=k.x̂, cov=k.Σ, gain=g)
end

"""
    predict(k::Kalman[, u])

Predict next state based on the model.
"""
function predict(k::Kalman, u)
    if size(u) != dimension_helper(newInstance).control_signal
        @warn "The dimension of the control_signal `u` or the signal mapping matrix `B` doesn't match."
    end
    k.x̂ = k.A * k.x̂ + k.B * k.u
    k.Σ = k.A * k.Σ * transpose(k.A) + k.Q
    (state=k.x̂, cov=k.Σ)
end
function predict(k::Kalman)
    k.x̂ = k.A * k.x̂
    k.Σ = k.A * k.Σ * transpose(k.A) + k.Q
    (state=k.x̂, cov=k.Σ)
end

end
