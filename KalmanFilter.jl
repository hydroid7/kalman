module KalmanFilter

using LinearAlgebra, Distributions, Printf

export Kalman, fusion, predict, K, next

mutable struct Kalman
    A # State transition
    Q # Process covariance matrix

    G # Measurement mapping
    R # Measurement covariance matrix

    x̂ # Current estimate mean
    Σ # Current estimate uncertianty

    B # Control signal mapping matrix
    u # Control signal

    Kalman(A, Q, G, R, x̂, Σ, B, u) = new(A, Q, G, R, x̂, Σ, B, u)
    Kalman(A, Q, G, R, x̂, Σ) = new(A, Q, G, R, x̂, Σ, I, I)
end

# Kalman Gain
function K(k::Kalman)
    K(k.Σ, k.G, k.R)
end
function K(Σ, G, R)
    Σ * transpose(G) * (G * Σ * transpose(G) + R)^-1
end

"""
    next(k::Kalman, y)

Compute a complete Kalman Step.
"""
function next(k::Kalman, y)
    newInstance = deepcopy(k)
    p = predict(newInstance).state
    f = fusion(newInstance, y)
    (model = newInstance, fusioned = f.state, predicted = p, gain = f.gain)
end

"""
    fusion(k::Kalman, y, [print = false])

Compute the filtered distribution.
"""
function fusion(k::Kalman, y)
    g = K(k.Σ, k.G, k.R)
    k.x̂ = k.x̂ + g * (y - k.G * k.x̂)
    k.Σ = (I - g * k.G) * k.Σ * transpose(I - g * k.G) + g * k.R * transpose(g)
    (state=k.x̂, cov=k.Σ, gain=g)
end

"""
    predict(k::Kalman[, print = false])

Predict next state without control signal.
"""
function predict(k::Kalman)
    k.x̂ = k.A * k.x̂
    k.Σ = k.A * k.Σ * transpose(k.A) + k.Q
    (state=k.x̂, cov=k.Σ)
end

# """
#     predict(k::Kalman, u[, print = false])
#
# Next prediction if there is a control signal.
# Optional parameter print for debugging.
# """
# function predict(k::Kalman, u, print = false)
#     k.x̂ = k.A * k.x̂ + k.B * u
#     k.Σ = k.A * k.Σ * transpose(k.A) + k.Q
#     if print
#         @printf "Predict: x̂ %.3f    Σ %.3f\n" k.x̂ k.Σ
#     end
#     (state=k.x̂, cov=k.Σ)
# end

end
