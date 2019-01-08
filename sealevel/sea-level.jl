
using Distributions, Compat, Plots, Random, CSV, DataFrames, GLM
# plotly()

data = CSV.File("/home/meszlenyilorant/bachelorarbeit/notebooks/sea-level.csv") |> DataFrame # Read csv
data[:GMSL] = convert(Array{Float64, 1}, data[:GMSL]) # Convert col to number

data[:Row] = collect(1:size(data)[1]) # Add row number for linear regression
describe(data)

first(data, 6)

# range = 0:270
plot(data[:Row], data[:GMSL], label = "Entwicklung der Meeresspiegel")

linearmodel = fit(LinearModel, @formula(GMSL ~ Row), data)

coef(linearmodel)
stderror(linearmodel)

f(x) = coef(linearmodel)[2] * x + coef(linearmodel)[1]
plot!(f, 1, size(data)[1], label = "Lineare Schätzung des Modells")

# Filtered state equations
x̂_F = (x̂, y, Σ, G, R) -> x̂ + Σ + transpose(G) * (G * Σ * transpose(G) + R)^-1 * (y - G * x̂)
Σ_F = (Σ, G, R) -> Σ - Σ * transpose(G) * (G * Σ * transpose(G) + R)^-1 * G * Σ

# Kalman Gain
K = (A, Σ, G, R) -> A * Σ * transpose(G) * (G * Σ * transpose(G) + R)^-1

# Predicted state equations
x̂_next = (A, x̂, K, y, G) -> A * x̂ + K * (y - G * x̂)
Σ_next = (A, Σ, K, G, Q) -> A * Σ * transpose(A) - K * G * Σ * transpose(A) + Q

mutable struct Kalman
    A
    G
    Q
    R

    x̂
    Σ
end

function fusion(k::Kalman, y)
    k.x̂ = x̂_F(k.x̂, y, k.Σ, k.G, k.R)
    k.Σ = Σ_F(k.Σ, k.G, k.R)
#     Normal(k.x̂, k.Σ^2)
end

function predict(k::Kalman, y)
    gain = K(k.A, k.Σ, k.G, k.R)
    k.x̂ = x̂_next(k.A, k.x̂, gain, y, k.G)
    k.Σ = Σ_next(k.A, k.Σ, gain, k.G, k.Q)
#     Normal(k.x̂, k.Σ^2)
end

A = coef(linearmodel)[2]
G = 1
Q = 0
R = stderror(linearmodel)[2]

x̂_0 = data[1, :GMSL]
Σ_0 = 0.6

y = (step) -> data[step, :GMSL] + rand(Normal(0, R))

model = Kalman(A, G, Q, R, x̂_0, Σ_0)

length = size(data)[1]
predicted = zeros(length)
fusioned = zeros(length)
confidence = zeros(length)
for i = 1:length
    y_cur = y(i)
    fusion(model, y_cur)
    fusioned[i] = model.x̂
    predict(model, y_cur)
    predicted[i] = model.x̂
    confidence[i] = model.Σ
end


# plot()
# plot!(predicted, label = "Vorhersage")
# plot!(data[:Row], data[:GMSL], label = "Wahrhafte Messung")
# plot!(fusioned, label = "Korrigierte Messung")
# plot!(data .- filtered)
