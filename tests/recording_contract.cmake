if(NOT DEFINED HEADLESS OR NOT DEFINED OUTPUT_ROOT)
  message(FATAL_ERROR "Expected HEADLESS and OUTPUT_ROOT")
endif()
string(RANDOM LENGTH 12 ALPHABET abcdef0123456789 run_suffix)
set(run_dir "${OUTPUT_ROOT}/${run_suffix}")
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 0.03 --grip-event 0.006:0.45 --out "${run_dir}"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Recording fixture failed: ${error}")
endif()
file(READ "${run_dir}/summary.json" summary)
string(JSON event_count GET "${summary}" parameter_events)
if(NOT event_count EQUAL 1)
  message(FATAL_ERROR "Expected one recorded revision change")
endif()
file(READ "${run_dir}/events.csv" events)
if(NOT events MATCHES "0.01,2,grip_mu,1,0.45")
  message(FATAL_ERROR "Off-grid event must apply at the next fixed boundary: ${events}")
endif()
file(READ "${run_dir}/metadata.json" metadata)
string(JSON schema GET "${metadata}" schema_version)
if(NOT schema EQUAL 18)
  message(FATAL_ERROR "New recordings must be schema 18")
endif()
# Judging (decision 0032): every run records how it was judged and what the judge saw, and --check-recording judges the
# recorded samples again.
string(JSON judged_discipline GET "${metadata}" competition discipline)
string(JSON judged_course GET "${metadata}" competition course)
if(NOT judged_discipline STREQUAL "trackdrive" OR NOT judged_course STREQUAL "laid along the track")
  message(FATAL_ERROR "A run is judged as a trackdrive on the course laid along its track: ${judged_discipline} ${judged_course}")
endif()
file(STRINGS "${run_dir}/timing.csv" timing_rows)
list(GET timing_rows 1 first_timing)
if(NOT first_timing MATCHES "^0,start,0[.]005[0-9]*,0,")
  message(FATAL_ERROR "The clock starts the tick the car leaves the start line: ${first_timing}")
endif()
file(STRINGS "${run_dir}/cones.csv" judged_cones)
list(LENGTH judged_cones judged_cone_lines)
if(judged_cone_lines LESS 200)
  message(FATAL_ERROR "A judged run records the course's cones whether or not it perceived them")
endif()
# Driving on cones (decision 0031): a run on the known track records the belief files' headers alone; a run on cones
# records what its driver believed, and --check-recording runs a driver again on the recorded readings and frames alone.
file(STRINGS "${run_dir}/beliefs.csv" bare_beliefs)
list(LENGTH bare_beliefs bare_belief_lines)
if(NOT bare_belief_lines EQUAL 1)
  message(FATAL_ERROR "A run on the known track records the beliefs header alone")
endif()
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 3 --track-width 5 --perception 6 --sensors 6 --drive cones
  --out "${run_dir}-cones" RESULT_VARIABLE result OUTPUT_VARIABLE coned ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT coned MATCHES "Cones: drove on the path believed")
  message(FATAL_ERROR "A run on cones failed: ${coned} ${error}")
endif()
file(READ "${run_dir}-cones/metadata.json" coned_metadata)
string(JSON coned_source GET "${coned_metadata}" cone_driving source)
if(NOT coned_source STREQUAL "measured")
  message(FATAL_ERROR "A run on cones with instruments believes their measurements, got ${coned_source}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-cones"
  RESULT_VARIABLE result OUTPUT_VARIABLE coned_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT coned_check MATCHES "believed and commanded the same at every decision")
  message(FATAL_ERROR "A run on cones must validate by a driver run again: ${coned_check} ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 1 --drive cones --out "${run_dir}-cones-blind"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR EXISTS "${run_dir}-cones-blind" OR NOT error MATCHES "needs --perception")
  message(FATAL_ERROR "Driving on cones without perception must be refused before any output: ${error}")
endif()
# Simulated cone perception (decision 0030): a run without it records the files' headers alone; a run with it records the
# cones and every frame, and --check-recording perceives the run again from the seed.
string(JSON bare_perception ERROR_VARIABLE bare_perception_error GET "${metadata}" perception)
if(NOT bare_perception_error)
  message(FATAL_ERROR "A run without --perception must record no perception")
endif()
execute_process(COMMAND "${HEADLESS}" --plant dynamic --laps 0 --duration 1 --perception 5 --out "${run_dir}-perception"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A perceived run failed: ${error}")
endif()
file(READ "${run_dir}-perception/metadata.json" perceived_metadata)
string(JSON perceived_seed GET "${perceived_metadata}" perception seed)
string(JSON perceived_rate GET "${perceived_metadata}" perception sensors 0 rate_hz)
if(NOT perceived_seed EQUAL 5 OR NOT perceived_rate EQUAL 10)
  message(FATAL_ERROR "The perception's seed and its sensor's rate must be recorded: ${perceived_seed} ${perceived_rate}")
endif()
file(STRINGS "${run_dir}-perception/perception_frames.csv" perceived_frames)
list(LENGTH perceived_frames perceived_frame_lines)
if(NOT perceived_frame_lines EQUAL 9)
  message(FATAL_ERROR "Eight frames a second in, the first two hundred milliseconds late: ${perceived_frame_lines} lines")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-perception"
  RESULT_VARIABLE result OUTPUT_VARIABLE perceived_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT perceived_check MATCHES "Perception: seed 5" OR NOT perceived_check MATCHES "nothing drove on them")
  message(FATAL_ERROR "A perceived run must validate and be reported: ${perceived_check} ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 0.02 --perception -1 --out "${run_dir}-bad-perception"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR EXISTS "${run_dir}-bad-perception" OR NOT error MATCHES "whole seed")
  message(FATAL_ERROR "A perception seed that is not whole must be refused before any output: ${error}")
endif()
# The car's own instruments (decision 0029): a run without them records an empty measurements.csv and no metadata
# block; a run with them records what each channel delivered and the seed it was drawn from, and --check-recording
# runs the same instruments again over the recorded states and requires the same stream.
string(JSON bare_sensors ERROR_VARIABLE bare_sensors_error GET "${metadata}" sensors)
if(NOT bare_sensors_error)
  message(FATAL_ERROR "A run without --sensors must record no instruments")
endif()
file(STRINGS "${run_dir}/measurements.csv" bare_measurements)
list(LENGTH bare_measurements bare_measurement_lines)
if(NOT bare_measurement_lines EQUAL 1)
  message(FATAL_ERROR "A run without instruments records the header alone, got ${bare_measurement_lines} lines")
endif()
execute_process(COMMAND "${HEADLESS}" --plant dynamic --laps 0 --duration 0.5 --sensors 99 --out "${run_dir}-sensors"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A measured run failed: ${error}")
endif()
file(READ "${run_dir}-sensors/metadata.json" measured_metadata)
string(JSON measured_seed GET "${measured_metadata}" sensors seed)
if(NOT measured_seed EQUAL 99)
  message(FATAL_ERROR "The seed every error came from must be recorded, got ${measured_seed}")
endif()
string(JSON measured_rate GET "${measured_metadata}" sensors pose rate_hz)
if(NOT measured_rate EQUAL 20)
  message(FATAL_ERROR "Each channel records its own rate, got ${measured_rate}")
endif()
file(STRINGS "${run_dir}-sensors/telemetry.csv" measured_telemetry)
list(LENGTH measured_telemetry measured_telemetry_lines)
file(STRINGS "${run_dir}-sensors/measurements.csv" measured_rows)
list(LENGTH measured_rows measured_row_lines)
if(NOT measured_row_lines EQUAL measured_telemetry_lines)
  message(FATAL_ERROR "One measurement per recorded tick, got ${measured_row_lines} for ${measured_telemetry_lines}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-sensors"
  RESULT_VARIABLE result OUTPUT_VARIABLE measured_check ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A measured run must validate: ${error}")
endif()
if(NOT measured_check MATCHES "Sensors: seed 99")
  message(FATAL_ERROR "The check must report the instruments the run carried: ${measured_check}")
endif()
if(NOT measured_check MATCHES "the car drove on its true state")
  message(FATAL_ERROR "The check must say the measurements drove nothing: ${measured_check}")
endif()
# Another seed measures the same run differently, which is what recording the seed is for.
execute_process(COMMAND "${HEADLESS}" --plant dynamic --laps 0 --duration 0.5 --sensors 100 --out "${run_dir}-sensors-other"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A measured run with another seed failed: ${error}")
endif()
file(READ "${run_dir}-sensors/measurements.csv" measured_first)
file(READ "${run_dir}-sensors-other/measurements.csv" measured_second)
if(measured_first STREQUAL measured_second)
  message(FATAL_ERROR "Another seed must measure the same run differently")
endif()
# A seed that is not a whole number is refused before anything is written.
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 0.02 --sensors 1.5 --out "${run_dir}-bad-seed"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR EXISTS "${run_dir}-bad-seed" OR NOT error MATCHES "whole seed")
  message(FATAL_ERROR "A seed that is not whole must be refused before any output: ${error}")
endif()
string(JSON default_controller GET "${metadata}" predictive_controller mode)
if(NOT default_controller STREQUAL "policy")
  message(FATAL_ERROR "A run must record the policy as its controller by default: ${default_controller}")
endif()
string(JSON default_local_planner GET "${metadata}" local_planner)
if(NOT default_local_planner STREQUAL "lattice")
  message(FATAL_ERROR "A run must record the lattice planner by default: ${default_local_planner}")
endif()
string(JSON default_plan GET "${metadata}" speed_plan mode)
if(NOT default_plan STREQUAL "grip fractions")
  message(FATAL_ERROR "A run planned with the grip fractions must say so: ${default_plan}")
endif()
# Control ticks at 0 and 0.02 s, plus the replan where the off-grid change applied at 0.01 s.
file(STRINGS "${run_dir}/decisions.csv" decision_rows)
list(LENGTH decision_rows decision_lines)
if(NOT decision_lines EQUAL 4 OR NOT decision_rows MATCHES "^decision,time_s,option" OR
   NOT decision_rows MATCHES ",cost_goal,profile_points,estimated_time_s,driven_by,controller_status,controller_iterations,controller_restarted,controller_note;[^;]*,straight,0,0,0,0,0,0,0,0,none,policy,none,0,0,;")
  message(FATAL_ERROR "Expected a header and three single-option decisions following the reference line: ${decision_rows}")
endif()
file(STRINGS "${run_dir}/paths.csv" path_rows)
file(STRINGS "${run_dir}/profiles.csv" profile_rows)
if(NOT path_rows STREQUAL "decision,option,point,s_m,offset_m" OR NOT profile_rows STREQUAL "decision,option,point,s_m,distance_m,curvature_1pm,speed_mps")
  message(FATAL_ERROR "Without a blockage paths.csv and profiles.csv hold only their headers: ${path_rows} ${profile_rows}")
endif()
file(STRINGS "${run_dir}/commands.csv" command_rows)
if(NOT command_rows STREQUAL "decision,option,point,time_s,acceleration_mps2,steering_rad")
  message(FATAL_ERROR "A run the policy drove holds no plan commands: ${command_rows}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}"
  RESULT_VARIABLE result OUTPUT_VARIABLE check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT check MATCHES "Decisions: 3 recorded" OR NOT check MATCHES "Local planner: lattice; 0 decisions passing" OR
   NOT check MATCHES "Controller: policy\n")
  message(FATAL_ERROR "The recorded decisions must validate and be reported: ${check} ${error}")
endif()
# The lattice planner (decision 0022) passes a lane blockage along recorded lattice paths; the five-offset planner is still
# selectable and recorded as such; an unknown planner, and a planner for the offline lattice, are refused before any output.
execute_process(COMMAND "${HEADLESS}" --obstruct 62:68:-4.5:1:cones --laps 0 --duration 8 --out "${run_dir}-lattice-pass"
  RESULT_VARIABLE result OUTPUT_VARIABLE lattice_run ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT lattice_run MATCHES "local planner: lattice")
  message(FATAL_ERROR "A lattice planner run must record: ${lattice_run} ${error}")
endif()
if(NOT lattice_run MATCHES "Planning: [1-9][0-9]* steps weighed alternatives; median [0-9.]+ ms")
  message(FATAL_ERROR "A run with alternatives must report its planning time: ${lattice_run}")
endif()
file(STRINGS "${run_dir}-lattice-pass/paths.csv" lattice_paths LIMIT_COUNT 3)
file(STRINGS "${run_dir}-lattice-pass/profiles.csv" lattice_profiles LIMIT_COUNT 3)
list(LENGTH lattice_paths lattice_path_lines)
list(LENGTH lattice_profiles lattice_profile_lines)
if(NOT lattice_path_lines EQUAL 3 OR NOT lattice_profile_lines EQUAL 3)
  message(FATAL_ERROR "Passing a blockage must record lattice paths and their speed profiles: ${lattice_paths} ${lattice_profiles}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-lattice-pass"
  RESULT_VARIABLE result OUTPUT_VARIABLE lattice_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT lattice_check MATCHES "Local planner: lattice; [1-9][0-9]* decisions passing along the lattice"
   OR NOT lattice_check MATCHES "decisions took the fastest of several clear actions by estimated time")
  message(FATAL_ERROR "The lattice run must validate and report its passes: ${lattice_check} ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --local-planner five-offsets --obstruct 62:68:-4.5:1:cones --laps 0 --duration 8 --out "${run_dir}-five-offsets"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A five-offset planner run must record: ${error}")
endif()
file(READ "${run_dir}-five-offsets/metadata.json" five_metadata)
string(JSON five_planner GET "${five_metadata}" local_planner)
file(STRINGS "${run_dir}-five-offsets/paths.csv" five_paths)
if(NOT five_planner STREQUAL "five offsets" OR NOT five_paths STREQUAL "decision,option,point,s_m,offset_m")
  message(FATAL_ERROR "A five-offset run must record its planner and no lattice paths: ${five_planner} ${five_paths}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-five-offsets"
  RESULT_VARIABLE result OUTPUT_VARIABLE five_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT five_check MATCHES "Local planner: five offsets[\r\n]" OR NOT five_check MATCHES "[1-9][0-9]* evaluated alternatives")
  message(FATAL_ERROR "The five-offset run must validate and report its planner: ${five_check} ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --local-planner fastest --laps 0 --duration 0.02 --out "${run_dir}-unknown-planner"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR EXISTS "${run_dir}-unknown-planner" OR NOT error MATCHES "lattice or five-offsets")
  message(FATAL_ERROR "An unknown local planner must be refused before any output: ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --local-planner lattice --write-lattice "${run_dir}-lattice-with-planner"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR EXISTS "${run_dir}-lattice-with-planner")
  message(FATAL_ERROR "A local planner has no effect on the offline lattice and must be refused before any output: ${error}")
endif()
file(READ "${run_dir}/plan-rev-2.csv" plan)
if(NOT plan MATCHES ",2,0.01")
  message(FATAL_ERROR "Recorded plan revision/time does not match parameter event")
endif()
file(SHA256 "${run_dir}/summary.json" before)
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 0.01 --out "${run_dir}"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0)
  message(FATAL_ERROR "A nonempty output directory must be rejected")
endif()
file(SHA256 "${run_dir}/summary.json" after)
if(NOT before STREQUAL after)
  message(FATAL_ERROR "Rejected output reuse mutated an existing run")
endif()
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 0.02 --grip-event 0.001:0.8 --grip-event 0.005:0.6 --out "${run_dir}-collision"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0)
  message(FATAL_ERROR "Events mapping to the same tick must be rejected before running")
endif()
execute_process(COMMAND "${HEADLESS}" --laps 0 --duration 0.02 --grip-event 0.005:0.8 --grip-event 0.006:0.6 --out "${run_dir}-distinct"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Events mapping to distinct ticks must remain valid: ${error}")
endif()
file(READ "${run_dir}-distinct/events.csv" distinct_events)
if(NOT distinct_events MATCHES "0.005,2,grip_mu,1,0.8" OR NOT distinct_events MATCHES "0.01,3,grip_mu,0.8,0.6")
  message(FATAL_ERROR "Event scheduling drift: ${distinct_events}")
endif()
# MAP steering is recorded with its table and validated; it is refused for the kinematic bicycle.
execute_process(COMMAND "${HEADLESS}" --plant dynamic --car soft-front --steering map --laps 0 --duration 0.05 --out "${run_dir}-map"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A MAP run must record: ${error}")
endif()
file(READ "${run_dir}-map/metadata.json" map_metadata)
string(JSON map_law GET "${map_metadata}" steering law)
string(JSON map_table GET "${map_metadata}" steering table_fingerprint)
if(NOT map_law STREQUAL "map" OR NOT map_table MATCHES "^[0-9a-f]+$")
  message(FATAL_ERROR "A MAP run must record its law and table fingerprint: ${map_law} ${map_table}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-map"
  RESULT_VARIABLE result OUTPUT_VARIABLE map_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT map_check MATCHES "Steering: MAP from steady-state steering table ${map_table} [(]this build derives it")
  message(FATAL_ERROR "The MAP recording must validate and report its table: ${map_check} ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --steering map --laps 0 --duration 0.02 --out "${run_dir}-kinematic-map"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-kinematic-map")
  message(FATAL_ERROR "MAP steering on the kinematic bicycle must be refused before any output")
endif()
# The four-wheel car records its wheels and load transfer and validates, and MAP steers it too.
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --brake-bias-front 0.4 --cg-height 0.42 --roll-balance-front 0.65
  --drag-area 0.8 --downforce-area 1.2 --aero-balance-front 0.45 --viscous-coupling 30
  --laps 0 --duration 0.05 --out "${run_dir}-wheels"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A four-wheel run must record: ${error}")
endif()
file(READ "${run_dir}-wheels/metadata.json" wheels_metadata)
string(JSON wheels_kind GET "${wheels_metadata}" vehicle_model kind)
string(JSON wheels_bias GET "${wheels_metadata}" vehicle_model brake_bias_front)
string(JSON wheels_height GET "${wheels_metadata}" vehicle_model cg_height_m)
string(JSON wheels_roll GET "${wheels_metadata}" vehicle_model roll_balance_front)
string(JSON wheels_drag GET "${wheels_metadata}" vehicle_model drag_area_m2)
string(JSON wheels_downforce GET "${wheels_metadata}" vehicle_model downforce_area_m2)
string(JSON wheels_aero GET "${wheels_metadata}" vehicle_model aero_balance_front)
string(JSON wheels_coupling GET "${wheels_metadata}" vehicle_model viscous_coupling_nms)
if(NOT wheels_drag EQUAL 0.8 OR NOT wheels_downforce EQUAL 1.2 OR NOT wheels_aero EQUAL 0.45 OR NOT wheels_coupling EQUAL 30)
  message(FATAL_ERROR "A four-wheel run must record its aerodynamics and coupling: ${wheels_drag} ${wheels_downforce} ${wheels_aero} ${wheels_coupling}")
endif()
if(NOT wheels_kind STREQUAL "four_wheel_car" OR NOT wheels_bias EQUAL 0.4 OR NOT wheels_height EQUAL 0.42 OR NOT wheels_roll EQUAL 0.65)
  message(FATAL_ERROR "A four-wheel run must record its car, brake bias and load transfer: ${wheels_kind} ${wheels_bias} ${wheels_height} ${wheels_roll}")
endif()
file(STRINGS "${run_dir}-wheels/telemetry.csv" wheels_header LIMIT_COUNT 1)
if(NOT wheels_header MATCHES "rear_right_wheel_speed_radps,front_left_wheel_load_n,front_right_wheel_load_n,rear_left_wheel_load_n,rear_right_wheel_load_n,longitudinal_acceleration_mps2$")
  message(FATAL_ERROR "Telemetry must end with the wheel speeds, loads and the acceleration achieved: ${wheels_header}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-wheels"
  RESULT_VARIABLE result OUTPUT_VARIABLE wheels_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT wheels_check MATCHES "Vehicle: four-wheel car with rotating wheels, load transfer and aerodynamics"
   OR NOT wheels_check MATCHES "centre of gravity 0.420 m high, roll balance front 0.650"
   OR NOT wheels_check MATCHES "drag area 0.800 m.2, downforce area 1.200 m.2, aero balance front 0.450, viscous coupling 30.000 N m s/rad")
  message(FATAL_ERROR "The four-wheel recording must validate and name its car: ${wheels_check} ${error}")
endif()
# The racing line (decision 0020): the preset's minimum-curvature line is driven and recorded with its corridor edges,
# and validates; a stated blockage with it is refused before any output. Without OSQP the flag is refused instead.
if(HAVE_RACELINE)
  execute_process(COMMAND "${HEADLESS}" --line racing --laps 0 --duration 1 --out "${run_dir}-racing-line"
    RESULT_VARIABLE result OUTPUT_VARIABLE racing_run ERROR_VARIABLE error)
  if(NOT result EQUAL 0 OR NOT racing_run MATCHES "Racing line: Foundry Circuit racing line, [0-9]+ iterations, converged")
    message(FATAL_ERROR "The racing line must be derived and driven: ${racing_run} ${error}")
  endif()
  file(READ "${run_dir}-racing-line/metadata.json" racing_metadata)
  string(JSON racing_name GET "${racing_metadata}" track_name)
  file(STRINGS "${run_dir}-racing-line/track.csv" racing_track LIMIT_COUNT 3)
  list(GET racing_track 0 racing_header)
  list(GET racing_track 2 racing_row)
  if(NOT racing_name STREQUAL "Foundry Circuit racing line" OR NOT racing_header MATCHES "left_edge_m,right_edge_m$" OR racing_row MATCHES ",5,5$")
    message(FATAL_ERROR "The racing line run must record the line and its corridor edges: ${racing_name} ${racing_header} ${racing_row}")
  endif()
  execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-racing-line" RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "A run on the racing line must validate: ${error}")
  endif()
  execute_process(COMMAND "${HEADLESS}" --line racing --obstruct 62:68:-4.5:1:cones --laps 0 --duration 1 --out "${run_dir}-racing-obstructed"
    RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
  if(result EQUAL 0 OR EXISTS "${run_dir}-racing-obstructed")
    message(FATAL_ERROR "A stated blockage on the racing line must be refused before any output")
  endif()
else()
  execute_process(COMMAND "${HEADLESS}" --line racing --laps 0 --duration 1 --out "${run_dir}-racing-line"
    RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
  if(result EQUAL 0 OR EXISTS "${run_dir}-racing-line" OR NOT error MATCHES "bootstrap-osqp")
    message(FATAL_ERROR "Without OSQP the racing line must be refused, saying how to build it: ${error}")
  endif()
endif()

# A centreline file is conditioned into the track the run drives and records (decision 0019): a 72-sided polygon
# around a 60 m circle, with a header and a comment. A figure eight is refused before any output.
file(WRITE "${run_dir}-polygon.csv" [=[# 72-sided polygon, 60 m radius
x_m,y_m
0.000,0.000
5.229,0.228
10.419,0.912
15.529,2.044
20.521,3.618
25.357,5.622
30.000,8.038
34.415,10.851
38.567,14.037
42.426,17.574
45.963,21.433
49.149,25.585
51.962,30.000
54.378,34.643
56.382,39.479
57.956,44.471
59.088,49.581
59.772,54.771
60.000,60.000
59.772,65.229
59.088,70.419
57.956,75.529
56.382,80.521
54.378,85.357
51.962,90.000
49.149,94.415
45.963,98.567
42.426,102.426
38.567,105.963
34.415,109.149
30.000,111.962
25.357,114.378
20.521,116.382
15.529,117.956
10.419,119.088
5.229,119.772
0.000,120.000
-5.229,119.772
-10.419,119.088
-15.529,117.956
-20.521,116.382
-25.357,114.378
-30.000,111.962
-34.415,109.149
-38.567,105.963
-42.426,102.426
-45.963,98.567
-49.149,94.415
-51.962,90.000
-54.378,85.357
-56.382,80.521
-57.956,75.529
-59.088,70.419
-59.772,65.229
-60.000,60.000
-59.772,54.771
-59.088,49.581
-57.956,44.471
-56.382,39.479
-54.378,34.643
-51.962,30.000
-49.149,25.585
-45.963,21.433
-42.426,17.574
-38.567,14.037
-34.415,10.851
-30.000,8.038
-25.357,5.622
-20.521,3.618
-15.529,2.044
-10.419,0.912
-5.229,0.228
]=])
execute_process(COMMAND "${HEADLESS}" --track "${run_dir}-polygon.csv" --track-width 8 --laps 0 --duration 1 --out "${run_dir}-conditioned"
  RESULT_VARIABLE result OUTPUT_VARIABLE conditioned_run ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT conditioned_run MATCHES "Track: [^\n]+-polygon conditioned from 72 points")
  message(FATAL_ERROR "A centreline file must be conditioned and driven: ${conditioned_run} ${error}")
endif()
file(READ "${run_dir}-conditioned/metadata.json" conditioned_metadata)
string(JSON conditioned_width GET "${conditioned_metadata}" track_width_m)
string(JSON conditioned_length GET "${conditioned_metadata}" track_length_m)
if(NOT conditioned_width EQUAL 8 OR conditioned_length LESS 370 OR conditioned_length GREATER 378)
  message(FATAL_ERROR "The run must record the conditioned track: width ${conditioned_width}, length ${conditioned_length}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-conditioned" RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A run on a conditioned track must validate: ${error}")
endif()
file(WRITE "${run_dir}-figure-eight.csv" "0,0\n40,30\n80,0\n40,-30\n0,0.5\n-40,30\n-80,0\n-40,-30\n")
execute_process(COMMAND "${HEADLESS}" --track "${run_dir}-figure-eight.csv" --laps 0 --duration 1 --out "${run_dir}-figure-eight-run"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE figure_error)
if(result EQUAL 0 OR EXISTS "${run_dir}-figure-eight-run" OR NOT figure_error MATCHES "crosses itself")
  message(FATAL_ERROR "A centreline that crosses itself must be refused before any output: ${figure_error}")
endif()
# Without a centreline file the width narrows the preset's own corridor, recorded as the run's track width.
execute_process(COMMAND "${HEADLESS}" --track-width 5 --laps 0 --duration 0.02 --out "${run_dir}-narrow-preset"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "The preset with a narrower corridor must run: ${error}")
endif()
file(READ "${run_dir}-narrow-preset/metadata.json" narrow_metadata)
string(JSON narrow_width GET "${narrow_metadata}" track_width_m)
if(NOT narrow_width EQUAL 5)
  message(FATAL_ERROR "The narrower corridor must be recorded: ${narrow_width}")
endif()
execute_process(COMMAND "${HEADLESS}" --track-width 50 --laps 0 --duration 0.02 --out "${run_dir}-too-wide-preset"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR EXISTS "${run_dir}-too-wide-preset" OR NOT error MATCHES "too wide")
  message(FATAL_ERROR "A corridor wider than the preset's bends allow must be refused before any output: ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --track-smoothing 0.2 --laps 0 --duration 0.02 --out "${run_dir}-smoothing-without-track"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-smoothing-without-track")
  message(FATAL_ERROR "Smoothing without a centreline file has no effect and must be refused before any output")
endif()

# The four-wheel car plans with its performance envelope through a grip change; the run records the envelope and its
# share, validates, and this build derives the recorded envelope. Other plants and a share without the envelope plan
# are refused before any output.
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --speed-plan envelope --envelope-fraction 0.9 --grip-event 0.5:0.9
  --laps 0 --duration 1 --out "${run_dir}-envelope-plan"
  RESULT_VARIABLE result OUTPUT_VARIABLE envelope_run ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT envelope_run MATCHES "speed plan: performance envelope [0-9a-f]+ at 0.90")
  message(FATAL_ERROR "An envelope plan run must record and report its envelope: ${envelope_run} ${error}")
endif()
file(READ "${run_dir}-envelope-plan/metadata.json" envelope_metadata)
string(JSON envelope_mode GET "${envelope_metadata}" speed_plan mode)
string(JSON envelope_print GET "${envelope_metadata}" speed_plan envelope_fingerprint)
string(JSON envelope_share GET "${envelope_metadata}" initial_config envelope_fraction)
if(NOT envelope_mode STREQUAL "performance envelope" OR NOT envelope_print MATCHES "^[0-9a-f]+$" OR NOT envelope_share EQUAL 0.9)
  message(FATAL_ERROR "An envelope plan run must record its mode, fingerprint and share: ${envelope_mode} ${envelope_print} ${envelope_share}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-envelope-plan"
  RESULT_VARIABLE result OUTPUT_VARIABLE envelope_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT envelope_check MATCHES "Speed plan: 0.90 of performance envelope ${envelope_print} [(]this build derives it")
  message(FATAL_ERROR "The envelope plan recording must validate and report its envelope: ${envelope_check} ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --plant dynamic --speed-plan envelope --laps 0 --duration 0.02 --out "${run_dir}-dynamic-envelope-plan"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-dynamic-envelope-plan")
  message(FATAL_ERROR "An envelope plan for the single-track car must be refused before any output")
endif()
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --envelope-fraction 0.9 --laps 0 --duration 0.02 --out "${run_dir}-share-without-envelope"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-share-without-envelope")
  message(FATAL_ERROR "An envelope share without the envelope plan has no effect and must be refused before any output")
endif()
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --steering map --laps 0 --duration 0.5 --out "${run_dir}-wheels-map"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "MAP steering on the four-wheel car must run: ${error}")
endif()
file(READ "${run_dir}-wheels-map/metadata.json" wheels_map_metadata)
string(JSON wheels_map_law GET "${wheels_map_metadata}" steering law)
string(JSON wheels_map_table GET "${wheels_map_metadata}" steering table_fingerprint)
if(NOT wheels_map_law STREQUAL "map" OR wheels_map_table STREQUAL "")
  message(FATAL_ERROR "The four-wheel car's MAP run must record its steering table: ${wheels_map_law} ${wheels_map_table}")
endif()
execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-wheels-map" RESULT_VARIABLE result OUTPUT_VARIABLE wheels_map_check ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT wheels_map_check MATCHES "Steering: MAP from steady-state steering table")
  message(FATAL_ERROR "The four-wheel car's MAP run must validate: ${wheels_map_check} ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --plant dynamic --cg-height 0.3 --laps 0 --duration 0.02 --out "${run_dir}-dynamic-height"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-dynamic-height")
  message(FATAL_ERROR "A centre of gravity height for a car without load transfer must be refused before any output")
endif()
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --cg-height 0.9 --laps 0 --duration 0.02 --out "${run_dir}-wheels-tall"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-wheels-tall")
  message(FATAL_ERROR "A centre of gravity above half the track must be refused before any output")
endif()
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --aero-balance-front 0.4 --laps 0 --duration 0.02 --out "${run_dir}-wheels-balance"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-wheels-balance")
  message(FATAL_ERROR "An aero balance without downforce must be refused before any output")
endif()
# Setup controls through the headless runner: mass scales yaw inertia with it; out-of-range and wrong-plant values are refused.
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --mass 1200 --max-power-kw 60 --laps 0 --duration 0.02 --out "${run_dir}-wheels-setup"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "A four-wheel setup run must record: ${error}")
endif()
file(READ "${run_dir}-wheels-setup/metadata.json" setup_metadata)
string(JSON setup_mass GET "${setup_metadata}" vehicle_model mass_kg)
string(JSON setup_inertia GET "${setup_metadata}" vehicle_model yaw_inertia_kgm2)
string(JSON setup_power GET "${setup_metadata}" vehicle_model max_drive_power_w)
if(NOT setup_mass EQUAL 1200 OR NOT setup_inertia EQUAL 2028 OR NOT setup_power EQUAL 60000)
  message(FATAL_ERROR "A setup run must record its mass, scaled yaw inertia and power: ${setup_mass} ${setup_inertia} ${setup_power}")
endif()
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --mass 2000 --laps 0 --duration 0.02 --out "${run_dir}-wheels-heavy"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-wheels-heavy")
  message(FATAL_ERROR "A mass outside the offered range must be refused before any output")
endif()
execute_process(COMMAND "${HEADLESS}" --plant dynamic --max-power-kw 60 --laps 0 --duration 0.02 --out "${run_dir}-dynamic-power"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-dynamic-power")
  message(FATAL_ERROR "A power limit for the single-track car must be refused before any output")
endif()
# The performance envelope is derived for the four-wheel car and written in TUM's format with its fingerprint.
execute_process(COMMAND "${HEADLESS}" --plant four-wheel --downforce-area 3 --write-envelope "${run_dir}-envelope"
  RESULT_VARIABLE result OUTPUT_VARIABLE envelope_output ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT envelope_output MATCHES "Envelope [0-9a-f]+: four-wheel car")
  message(FATAL_ERROR "A four-wheel envelope must be derived and reported: ${envelope_output} ${error}")
endif()
string(REGEX MATCH "Envelope ([0-9a-f]+)" envelope_match "${envelope_output}")
set(envelope_fingerprint "${CMAKE_MATCH_1}")
foreach(envelope_file ggv.csv ax_max_machines.csv envelope.csv)
  file(READ "${run_dir}-envelope/${envelope_file}" envelope_text)
  if(NOT envelope_text MATCHES "${envelope_fingerprint}")
    message(FATAL_ERROR "${envelope_file} must name the envelope fingerprint ${envelope_fingerprint}")
  endif()
endforeach()
file(STRINGS "${run_dir}-envelope/ggv.csv" ggv_rows REGEX "^[0-9]")
list(LENGTH ggv_rows ggv_count)
if(NOT ggv_count EQUAL 11)
  message(FATAL_ERROR "ggv.csv must have a row per speed from 4 to 24 m/s: ${ggv_count}")
endif()
execute_process(COMMAND "${HEADLESS}" --plant dynamic --write-envelope "${run_dir}-dynamic-envelope"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-dynamic-envelope")
  message(FATAL_ERROR "An envelope for the single-track car must be refused before any output")
endif()
execute_process(COMMAND "${HEADLESS}" --plant dynamic --viscous-coupling 20 --laps 0 --duration 0.02 --out "${run_dir}-dynamic-coupling"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-dynamic-coupling")
  message(FATAL_ERROR "A viscous coupling for the single-track car must be refused before any output")
endif()

# The offline lattice (decision 0021) is laid along the line a run would drive and written without a lap.
execute_process(COMMAND "${HEADLESS}" --write-lattice "${run_dir}-lattice"
  RESULT_VARIABLE result OUTPUT_VARIABLE lattice_output ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT lattice_output MATCHES "Lattice: Foundry Circuit, [0-9.]+ m; ([0-9]+) layers, ([0-9]+) nodes, ([0-9]+) edges kept of ([0-9]+)")
  message(FATAL_ERROR "The preset's lattice must be laid and reported: ${lattice_output} ${error}")
endif()
set(lattice_layers "${CMAKE_MATCH_1}")
set(lattice_nodes "${CMAKE_MATCH_2}")
set(lattice_edges "${CMAKE_MATCH_3}")
foreach(lattice_table layers nodes edges)
  file(STRINGS "${run_dir}-lattice/${lattice_table}.csv" lattice_rows)
  list(LENGTH lattice_rows lattice_count)
  math(EXPR lattice_count "${lattice_count} - 1")
  if(NOT lattice_count EQUAL "${lattice_${lattice_table}}")
    message(FATAL_ERROR "${lattice_table}.csv must hold a row per reported item: ${lattice_count} against ${lattice_${lattice_table}}")
  endif()
endforeach()
file(READ "${run_dir}-lattice/lattice.json" lattice_json)
string(JSON lattice_json_edges GET "${lattice_json}" edges)
string(JSON lattice_json_track GET "${lattice_json}" track_name)
if(NOT lattice_json_edges EQUAL lattice_edges OR NOT lattice_json_track STREQUAL "Foundry Circuit" OR NOT EXISTS "${run_dir}-lattice/edge_samples.csv")
  message(FATAL_ERROR "lattice.json must describe the lattice written: ${lattice_json}")
endif()
execute_process(COMMAND "${HEADLESS}" --write-lattice "${run_dir}-lattice" RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR NOT error MATCHES "not empty")
  message(FATAL_ERROR "A lattice must not be written over an existing one: ${error}")
endif()
execute_process(COMMAND "${HEADLESS}" --obstruct 62:68:-4.5:1:cones --write-lattice "${run_dir}-lattice-obstructed"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0 OR EXISTS "${run_dir}-lattice-obstructed")
  message(FATAL_ERROR "A stated blockage has no effect on the offline lattice and must be refused before any output")
endif()
if(HAVE_RACELINE)
  execute_process(COMMAND "${HEADLESS}" --line racing --write-lattice "${run_dir}-lattice-racing-line"
    RESULT_VARIABLE result OUTPUT_VARIABLE lattice_output ERROR_VARIABLE error)
  if(NOT result EQUAL 0 OR NOT lattice_output MATCHES "Lattice: Foundry Circuit racing line")
    message(FATAL_ERROR "The racing line's lattice must be laid along it: ${lattice_output} ${error}")
  endif()
  file(READ "${run_dir}-lattice-racing-line/lattice.json" lattice_json)
  string(JSON lattice_edges_recorded GET "${lattice_json}" corridor_edges)
  if(NOT lattice_edges_recorded)
    message(FATAL_ERROR "The racing line's lattice must be laid within its corridor's edges: ${lattice_json}")
  endif()
endif()
# Model predictive contouring control (decision 0024) drives the chosen action when OSQP is built: the run records MPCC as its
# controller with its settings, decisions driven by MPCC with their plans, and a solve-time distribution is printed; it is
# refused for the offline lattice and, without OSQP, before any output.
if(HAVE_MPCC)
  execute_process(COMMAND "${HEADLESS}" --plant dynamic --controller mpcc --obstruct 62:68:-4.5:1:cones --laps 0 --duration 6 --out "${run_dir}-mpcc"
    RESULT_VARIABLE result OUTPUT_VARIABLE mpcc_run ERROR_VARIABLE error)
  if(NOT result EQUAL 0 OR NOT mpcc_run MATCHES "controller: mpcc" OR
     NOT mpcc_run MATCHES "Controller: MPCC drove [1-9][0-9]* of [0-9]+ decisions.*Solving: median [0-9.]+ ms, 95th percentile")
    message(FATAL_ERROR "An MPCC run must record and report its solving: ${mpcc_run} ${error}")
  endif()
  file(READ "${run_dir}-mpcc/metadata.json" mpcc_metadata)
  string(JSON mpcc_mode GET "${mpcc_metadata}" predictive_controller mode)
  string(JSON mpcc_stages GET "${mpcc_metadata}" predictive_controller settings stages)
  if(NOT mpcc_mode STREQUAL "mpcc" OR NOT mpcc_stages EQUAL 60)
    message(FATAL_ERROR "The run must record MPCC and its settings: ${mpcc_mode} ${mpcc_stages}")
  endif()
  execute_process(COMMAND "${HEADLESS}" --check-recording "${run_dir}-mpcc" RESULT_VARIABLE result OUTPUT_VARIABLE mpcc_check ERROR_VARIABLE error)
  if(NOT result EQUAL 0 OR NOT mpcc_check MATCHES "Controller: mpcc \\(60 stages of 0.050 s\\); it drove [1-9]" OR
     NOT mpcc_check MATCHES "Model error: the recorded plant under its plan's commands from its plan's rear axle; 0.5 s ahead median [0-9.]+ m")
    message(FATAL_ERROR "The MPCC run must validate and report who drove and its model error: ${mpcc_check} ${error}")
  endif()
  file(STRINGS "${run_dir}-mpcc/commands.csv" mpcc_commands LIMIT_COUNT 3)
  list(LENGTH mpcc_commands mpcc_command_lines)
  if(NOT mpcc_command_lines EQUAL 3)
    message(FATAL_ERROR "The MPCC run records the commands of the plans that drove: ${mpcc_commands}")
  endif()
  execute_process(COMMAND "${HEADLESS}" --prediction-error "${run_dir}-mpcc" RESULT_VARIABLE result OUTPUT_VARIABLE mpcc_error ERROR_VARIABLE error)
  if(NOT result EQUAL 0 OR NOT mpcc_error MATCHES "ahead_s,plans_reached,plan_median_m" OR NOT mpcc_error MATCHES "\n3.00,[0-9]+,")
    message(FATAL_ERROR "The prediction error is reported every 0.25 s to the horizon: ${mpcc_error} ${error}")
  endif()
  execute_process(COMMAND "${HEADLESS}" --prediction-error "${run_dir}" RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
  if(result EQUAL 0 OR NOT error MATCHES "driven by the policy")
    message(FATAL_ERROR "A run the policy drove has no prediction error to report: ${error}")
  endif()
  execute_process(COMMAND "${HEADLESS}" --controller mpcc --laps 0 --duration 0.02 --out "${run_dir}-mpcc-kinematic"
    RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
  if(result EQUAL 0 OR EXISTS "${run_dir}-mpcc-kinematic" OR NOT error MATCHES "no tires")
    message(FATAL_ERROR "MPCC for the kinematic bicycle must be refused before any output: ${error}")
  endif()
  execute_process(COMMAND "${HEADLESS}" --controller mpcc --write-lattice "${run_dir}-lattice-with-controller"
    RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
  if(result EQUAL 0 OR EXISTS "${run_dir}-lattice-with-controller" OR NOT error MATCHES "no effect on the offline lattice")
    message(FATAL_ERROR "A controller for the offline lattice must be refused: ${error}")
  endif()
else()
  execute_process(COMMAND "${HEADLESS}" --controller mpcc --laps 0 --duration 0.02 --out "${run_dir}-no-mpcc"
    RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
  if(result EQUAL 0 OR EXISTS "${run_dir}-no-mpcc" OR NOT error MATCHES "no MPCC")
    message(FATAL_ERROR "Without OSQP an MPCC run must be refused: ${error}")
  endif()
endif()
execute_process(COMMAND "${HEADLESS}" --controller fastest --laps 0 --duration 0.02 --out "${run_dir}-unknown-controller"
  RESULT_VARIABLE result OUTPUT_QUIET ERROR_VARIABLE error)
if(result EQUAL 0 OR EXISTS "${run_dir}-unknown-controller" OR NOT error MATCHES "policy or mpcc")
  message(FATAL_ERROR "An unknown controller must be refused: ${error}")
endif()
