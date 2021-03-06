using Plots
Plots.scalefontsizes(1.6)
x = collect(0:0.01:7.5)
f1(x) = x > 5 ? x^2 : -2x+35
f2(x) = 0.05x^2

plot(xlim = (2.5, 7.5), title = "Verleich zweier nichtlinearer Systeme", dpi = 350)
plot!(x, f1.(x), label = "Lineare und Quadratische Funktion")
plot!(x, f2.(x), label = "Polynom 2. Grades")
png("plot-nichtlineares-system.png")

x = y = z = [1, 2, 3]

plot(title = "Zustände in einem 3D Zustandsraum", dpi = 350)
scatter!(x, y, z, label = "Zustand")
png("plot-3d-system.png")