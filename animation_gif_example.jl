model = Kalman(A, Q, G, R, x_0, Σ_0)
y(i) = [m_x[i]; m_y[i]]
predicted = zeros(Float64, 4, n)
fusioned = zeros(Float64, 4, n)
confidence = zeros(Float64, 4, 4, n)
gains = zeros(Float64, 2, 4, n)
for i = 1:n
    y_cur = y(i)
    fusion(model, y_cur)
    fusioned[:, i] = model.x̂
    predict(model)
    predicted[:, i] = model.x̂
#     confidence[i] = model.Σ
    gains[:, :, i] = reshape(K(model), (2, 4, 1))
end
output = DataFrame(fusioned', [:pX, :pY, :vX, :vY])

plt = plot(1, xlim = (0, 400), ylim = (0, 200), label = "Gefilterte Laufbahn")
plot!((x) -> 0.5x, linestyle = :dot, seriestype = :line, label = "Theoretische Laufbahn")
@gif for i = eachrow(output)
    push!(plt, i.pX, i.pY)
end
