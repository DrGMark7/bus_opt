# make_tc_massive.jl
using JSON

function make_tc_massive(; buses=6, cei_n=2000, t2_n=1000, tau=30, c_max=20, w_max=60, horizon=1100)
    B = collect(1:buses)

    # CEI: 2 pax/min for minutes 0..999 => 2000 pax
    arr_cei = Dict{String,Int}()
    pid = 1
    for m in 0:999
        if pid <= cei_n
            arr_cei[string(pid)] = m; pid += 1
        end
        if pid <= cei_n
            arr_cei[string(pid)] = m; pid += 1
        end
    end
    while pid <= cei_n
        arr_cei[string(pid)] = 999
        pid += 1
    end

    # T2: 1 pax/min for minutes 0..999 => 1000 pax
    arr_t2 = Dict{String,Int}()
    for p in 1:t2_n
        arr_t2[string(p)] = (p - 1) % 1000
    end

    cfg = Dict(
        "T" => "0:$(horizon)",
        "B" => B,
        "L" => 1,
        "tau" => tau,
        "c_max" => c_max,
        "w_max" => w_max,
        "arrivals" => Dict("CEI" => arr_cei, "T2" => arr_t2),
        "initial_positions" => [
            Dict("bus"=>1, "terminal"=>"CEI"),
            Dict("bus"=>2, "terminal"=>"CEI"),
            Dict("bus"=>3, "terminal"=>"CEI"),
            Dict("bus"=>4, "terminal"=>"T2"),
            Dict("bus"=>5, "terminal"=>"T2"),
            Dict("bus"=>6, "terminal"=>"T2")
        ]
    )

    open("tc_massive.json", "w") do io
        write(io, JSON.json(cfg))
    end
    println("Wrote tc_massive.json")
end

make_tc_massive()
