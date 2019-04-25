using UnicodePlots

const points = 20

function command(from, to)
    start = "HEAD" * "^"^from
    last = "HEAD" * "^"^to
    `git diff --numstat $start $last`
end
data = zeros(points, 3)

for i = collect(1:points)
    res = split(read(command(i - 1, i), String), "\t")

    if size(res)[1] > 3
        data[i, :] = [i parse(Float64, res[1]) parse(Float64, res[2])]
    end
end

p = barplot(["Add", "Delete"],
        [sum(data[:, 2]), sum(data[:, 3])],
        title = "Changed Lines over the last $points commits.")
println(p)

p = lineplot(data[:, 1], data[:, 2], title = "LoC", name = "Addition", xlabel = "Commits", ylabel = "LoC")
lineplot!(p, data[:, 1], data[:, 3], color = :blue, name = "Deletion")

println(p)
