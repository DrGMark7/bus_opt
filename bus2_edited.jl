using JSON
using Random
using JuMP
using CPLEX

Random.seed!(42)
#! Note หา Test case
# Parameters
T    = 0:780                # time slots (integer minutes)
B    = 1:1                  # buses
P    = 1:10                 # passengers at CEI
P_r  = 1:5                 # passengers at CRBT2

L      = 1                   # time step
tau    = 25                  # travel time between terminals
c_max  = 5                 # bus capacity
w_max  = 600                 # max waiting time (minutes)

# Random arrivals (ensure feasible windows)
latest_board_time = last(T) - tau
safe_arr_upper    = max(first(T), latest_board_time - w_max)
arr   = Dict(p  => rand(first(T):safe_arr_upper) for p  in P)
arr_r = Dict(pr => rand(first(T):safe_arr_upper) for pr in P_r)
# arr   = Dict(p  => 0 for p  in P)
# arr_r = Dict(pr => 0 for pr in P_r)

println("arr  (CEI) sample: ", collect(arr))
println("arr_r(CRBT2) sample: ", collect(arr_r))

# Build feasible time windows per passenger (restrict domains)
Tmax_board = last(T) - tau

# time window for CEI passenger i
T_i  = Dict{Int, Vector{Int}}()
for i in P
    lo = arr[i]
    hi = min(arr[i] + w_max, Tmax_board)
    if lo <= hi
        T_i[i] = collect(lo:hi)
    else
        # Should not happen due to safe_arr_upper, but guard anyway
        T_i[i] = Int[]
    end
end

# time window for CRBT2 passenger i_r
T_ir = Dict{Int, Vector{Int}}()
for i in P_r
    lo = arr_r[i]
    hi = min(arr_r[i] + w_max, Tmax_board)
    if lo <= hi
        T_ir[i] = collect(lo:hi)
    else
        T_ir[i] = Int[]
    end
end

# Model
model = Model(CPLEX.Optimizer)
set_optimizer_attribute(model, "CPX_PARAM_THREADS", 20)

# Decision variables

# Assignment (restricted domains per passenger)
# Note: Use containers to define sparse indexing
@variable(model, x[i in P, c in B, t in T_i[i]], Bin)     # CEI -> CRBT2
@variable(model, x_r[i in P_r, c in B, t in T_ir[i]], Bin) # CRBT2 -> CEI

# Bus availability and departure decisions
@variable(model, ba[t in T, c in B],   Bin)   # bus available at CEI at time t
@variable(model, ba_r[t in T, c in B], Bin)   # bus available at CRBT2 at time t
@variable(model, bd[t in T, c in B],   Bin)   # depart from CEI at t
@variable(model, bd_r[t in T, c in B], Bin)   # depart from CRBT2 at t

# Objective: minimize total waiting time (no Big-M, no Dep, no Z)
@objective(model, Min,
    sum((t - arr[i])   * x[i,c,t]   for i in P,   c in B, t in T_i[i]) +
    sum((t - arr_r[i]) * x_r[i,c,t] for i in P_r, c in B, t in T_ir[i])
)

# 1) Each passenger assigned exactly once
@constraint(model, [i in P],   sum(x[i,c,t]   for c in B, t in T_i[i])   == 1)
@constraint(model, [i in P_r], sum(x_r[i,c,t] for c in B, t in T_ir[i]) == 1)

# 2) Capacity and gating by departures (remove redundant caps)
@constraint(model, [t in T, c in B], sum(x[i,c,t]   for i in P if t in T_i[i])   <= c_max * bd[t,c])
@constraint(model, [t in T, c in B], sum(x_r[i,c,t] for i in P_r if t in T_ir[i]) <= c_max * bd_r[t,c])

# Link assignment to departures (x ≤ bd)
@constraint(model, [i in P,   c in B, t in T_i[i]],   x[i,c,t]   <= bd[t,c])
@constraint(model, [i in P_r, c in B, t in T_ir[i]], x_r[i,c,t] <= bd_r[t,c])

# 3) Bus must exist at terminal to depart
@constraint(model, [t in T, c in B], bd[t,c]   <= ba[t,c])
@constraint(model, [t in T, c in B], bd_r[t,c] <= ba_r[t,c])

# 4) Cannot be at both terminals simultaneously
@constraint(model, [t in T, c in B], ba[t,c] + ba_r[t,c] <= 1)

# 5) Start-of-day: bus available at exactly one terminal
@constraint(model, [c in B], ba[first(T), c] + ba_r[first(T), c] == 1)

# 6) No boarding in the last τ time slots: already excluded via T_i/T_ir upper bound
#    But still forbid bus departures that cannot complete trip if desired (optional).
#    If you want to forbid departures after Tmax_board at both terminals:

for t in (Tmax_board+1):last(T), c in B
    fix(bd[t,c], 0.0; force=true)
    fix(bd_r[t,c], 0.0; force=true)
end
"""

for t in T, c in B
    if t % tau != 0
        fix(bd[t,c], 0.0; force=true)
        fix(bd_r[t,c], 0.0; force=true)
    end
end
"""

# 7) Flow equalities of bus state (same logic as before)
@constraint(model, [t in T, c in B], bd[t,c] + bd_r[t,c] <= 1)


for c in B
    for t in T[1:end-1]   # every t < last(T)
        if t >= tau
            @constraint(model, ba[t, c]   == ba[t - 1, c]   - bd[t - 1, c]   + bd_r[t - tau, c])
            @constraint(model, ba_r[t, c] == ba_r[t - 1, c] - bd_r[t - 1, c] + bd[t - tau, c])
        	
	"""
        else
            @constraint(model, ba[t+1, c]   == ba[t, c]   - bd[t, c])
            @constraint(model, ba_r[t+1, c] == ba_r[t, c] - bd_r[t, c])
	"""
	end
    end
end

for c in B
    for t in 1:tau-1
	@constraint(model, ba[t,c]   == ba[t-1,c] - bd[t-1,c])
        @constraint(model, ba_r[t,c] == ba_r[t-1,c] - bd_r[t-1,c])
    end
end


@constraint(model, bd[0, 1] == 1)

"""
@constraint(model, bd_r[25, 1] == 1)
"""

# CPLEX tuning (optional but helpful)
set_optimizer_attribute(model, "CPX_PARAM_MIPEMPHASIS", 1) # feasibility
set_optimizer_attribute(model, "CPX_PARAM_HEURFREQ", 10)
set_optimizer_attribute(model, "CPX_PARAM_EPGAP", 0.02)    # 2% gap
# set_optimizer_attribute(model, "CPX_PARAM_TILIM", 300)   # time limit (sec), optional

optimize!(model)

println("\nStatus: ", termination_status(model))
if termination_status(model) == MOI.OPTIMAL || termination_status(model) == MOI.FEASIBLE_POINT
    println("Objective value (total waiting): ", objective_value(model))

    println("\nDepartures from CEI (bd[t]=1):")
    for t in T, c in B
        if value(bd[t,c]) > 0.5
       	 println("  t=$t, c=$c bd=", value(bd[t,c]))
        end
    end

    println("\nDepartures from CRBT2 (bd_r[t]=1):")
    for t in T, c in B
        if value(bd_r[t,c]) > 0.5
       	 println("  t=$t, c=$c bd_r=", value(bd_r[t,c]))
        end
    end

    println("\nAssignments (CEI):")
    """
    let shown = 0
    """
    for i in P, c in B, t in T_i[i]
        if value(x[i,c,t]) > 0.5
            wait_i = t - arr[i]
            println("  i=$i -> t=$t (CEI), wait=", wait_i)
                """
		shown += 1
                if shown >= 20
                    break
                end
		"""
        end
    end
    """	
    end
    """
    println("\nAssignments (CRBT2):")
    for i in P_r, c in B, t in T_ir[i]
        if value(x_r[i,c,t]) > 0.5
            wait_ir = t - arr_r[i]
            println("  i_r=$i -> t=$t (CRBT2), wait=", wait_ir)
        end
    end

    println("\nInitial availability:")
    for c in B
        println("  ba[0,$c]   = ", value(ba[first(T),c]))
        println("  ba_r[0,$c] = ", value(ba_r[first(T),c]))
    end
else
    println("Model not optimal. Consider loosening w_max or expanding T.")
end


function result_payload()
    # meta
    meta = Dict(
        "T_start" => first(T), "T_end" => last(T), "L" => L,
        "tau" => tau, "capacity" => c_max, "w_max" => w_max
    )

    # sets
    sets = Dict(
        "buses" => collect(B),
        "P_CEI" => collect(P),
        "P_T2"  => collect(P_r)
    )

    # arrivals
    arr_cei = [Dict("p"=>i, "arr"=>arr[i])   for i in P]
    arr_t2  = [Dict("p"=>i, "arr"=>arr_r[i]) for i in P_r]

    # initial positions from ba[0,*] / ba_r[0,*]
    initpos = Vector{Dict{String,Any}}()
    for c in B
        b0  = value(ba[first(T), c])
        b0r = value(ba_r[first(T), c])
        if b0 > 0.5 && b0r < 0.5
            push!(initpos, Dict("bus"=>c, "terminal"=>"CEI"))
        elseif b0r > 0.5 && b0 < 0.5
            push!(initpos, Dict("bus"=>c, "terminal"=>"T2"))
        else
            # หากโมเดลกำหนดไม่ชัด ให้ fallback จาก constraint เริ่มวัน
            term = b0 >= b0r ? "CEI" : "T2"
            push!(initpos, Dict("bus"=>c, "terminal"=>term))
        end
    end

    # departures (collect only bd==1 / bd_r==1)
    deps = Vector{Dict{String,Any}}()
    for t in T, c in B
        if value(bd[t,c]) > 0.5
            push!(deps, Dict("terminal"=>"CEI", "bus"=>c, "t"=>t))
        end
        if value(bd_r[t,c]) > 0.5
            push!(deps, Dict("terminal"=>"T2", "bus"=>c, "t"=>t))
        end
    end

    # assignments (x==1 / x_r==1)
    asg = Vector{Dict{String,Any}}()
    for i in P, c in B, t in T_i[i]
        if value(x[i,c,t]) > 0.5
            push!(asg, Dict("terminal"=>"CEI", "p"=>i, "bus"=>c, "t"=>t, "wait"=>(t - arr[i])))
        end
    end
    for i in P_r, c in B, t in T_ir[i]
        if value(x_r[i,c,t]) > 0.5
            push!(asg, Dict("terminal"=>"T2", "p"=>i, "bus"=>c, "t"=>t, "wait"=>(t - arr_r[i])))
        end
    end

    return Dict(
        "schema_version" => 1,
        "meta" => meta,
        "status" => string(termination_status(model)),
        "objective" => (isfinite(objective_value(model)) ? objective_value(model) : nothing),
        "sets" => sets,
        "arrivals" => Dict("CEI"=>arr_cei, "T2"=>arr_t2),
        "initial_positions" => initpos,
        "departures" => deps,
        "assignments" => asg
    )
end

if termination_status(model) in (MOI.OPTIMAL, MOI.FEASIBLE_POINT)
    payload = result_payload()
    # Dump as File
    open("result.json", "w") do io
        write(io, JSON.json(payload))  # one-line JSON
    end
    # println(JSON.json(payload))
else
    # if infeasible unless raise meta+status for checking
    payload = Dict(
        "schema_version"=>1, "meta"=>Dict("T_start"=>first(T),"T_end"=>last(T),"L"=>L,"tau"=>tau,"capacity"=>c_max,"w_max"=>w_max),
        "status"=>string(termination_status(model)), "objective"=>nothing,
        "sets"=>Dict("buses"=>collect(B),"P_CEI"=>collect(P),"P_T2"=>collect(P_r)),
        "arrivals"=>Dict("CEI"=>[], "T2"=>[]),
        "initial_positions"=>[], "departures"=>[], "assignments"=>[]
    )
    open("result_test.json","w") do io; write(io, JSON.json(payload)); end
end
