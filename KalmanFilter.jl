module KalmanFilter

using LinearAlgebra, Distributions, Printf

export Kalman, fusion, predict, K

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
    Kalman(A, Q, G, R, x̂, Σ) = new(A, Q, G, R, x̂, Σ, I, 1)
end

# Kalman Gain
function K(k::Kalman)
    K(k.Σ, k.G, k.R)
end
function K(Σ, G, R)
    Σ * transpose(G) * (G * Σ * transpose(G) + R)^-1
end

"""
    fusion(k::Kalman, y, [print = false])

Compute the filtered distribution.
"""
function fusion(k::Kalman, y, print = false)
    gain = K(k.Σ, k.G, k.R)
    k.x̂ = k.x̂ + gain * (y - k.G * k.x̂)
    k.Σ = (I - gain * k.G) * k.Σ * transpose(I - gain * k.G) + gain * k.R * transpose(gain)
    if print
        @printf "Fusion:  x̂ %2.3f    Σ %2.3f    K %.3f\n" gain k.x̂ k.Σ
    end
    (state=k.x̂, cov=k.Σ)
end

"""
    predict(k::Kalman[, print = false])

Predict next state without control signal.
"""
function predict(k::Kalman, print = false)
    k.x̂ = k.A * k.x̂
    k.Σ = k.A * k.Σ * transpose(k.A) + k.Q
    if print
        @printf "Predict: x̂ %.3f    Σ %.3f\n" k.x̂ k.Σ
    end
    (state=k.x̂, cov=k.Σ)
end

"""
    predict(k::Kalman, u[, print = false])

Next prediction if there is a control signal.
Optional parameter print for debugging.
"""
function predict(k::Kalman, u, print = false)
    k.x̂ = k.A * k.x̂ + k.B * u
    k.Σ = k.A * k.Σ * transpose(k.A) + k.Q
    if print
        @printf "Predict: x̂ %.3f    Σ %.3f\n" k.x̂ k.Σ
    end
    (state=k.x̂, cov=k.Σ)
end

end
