module KalmanFilter

using LinearAlgebra, Distributions

export Kalman, fusion, predict, K

mutable struct Kalman
    A # State transition
    Q # Process covariance matrix

    G # Measurement mapping
    R # Measurement covariance matrix

    x̂ # Current estimate mean
    Σ # Current estimate uncertianty
end

t = (m) -> transpose(m) # Sorthand for transpose
# Filtered state equations
# x̂_F = (x̂, y, Σ, G, R) -> x̂ + Σ * t(G) * (G * Σ * t(G) + R)^-1 * (y - G * x̂)
# Σ_F = (Σ, G, R) -> Σ - Σ * t(G) * (G * Σ * t(G) + R)^-1 * G * Σ

# Kalman Gain
function K(k::Kalman)
    K(k.Σ, k.G, k.R)
end
function K(Σ, G, R)
    Σ * t(G) * (G * Σ * t(G) + R)^-1
end
# Predicted state equations
# x̂_next = (A, x̂, K, y, G) -> A * x̂ + K * (y - G * x̂)
# Σ_next = (A, Σ, K, G, Q) -> A * Σ * t(A) - K * G * Σ * t(A) + Q


"""
Compute the filtered distribution.
"""
function fusion(k::Kalman, y)
    # k.x̂ = x̂_F(k.x̂, y, k.Σ, k.G, k.R)
    # k.Σ = Σ_F(k.Σ, k.G, k.R)
    # Normal(k.x̂, k.Σ)
    gain = K(k.Σ, k.G, k.R)
    k.x̂ = k.x̂ + gain * (y - k.G * k.x̂)
    k.Σ = (I - gain * k.G) * k.Σ
end

"""
Predict next state.
"""
function predict(k::Kalman)
#    gain = K(k.A, k.Σ, k.G, k.R)
#    k.x̂ = x̂_next(k.A, k.x̂, gain, y, k.G)
#    k.Σ = Σ_next(k.A, k.Σ, gain, k.G, k.Q)
#    Normal(k.x̂, k.Σ)
    k.x̂ = k.A * k.x̂
    k.Σ = k.A * k.Σ * t(k.A) + k.Q
    # Normal(k.x̂, k.Σ)
end

end
