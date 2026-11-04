onerror {quit -force -code 1}
set project_dir [file normalize [pwd]]
set test_dir [file join $project_dir tests]
set local_ini [file join $project_dir modelsim.ini]
if {[file exists [file join $test_dir work_autoswing_handoff]]} {
    vdel -lib [file join $test_dir work_autoswing_handoff] -all
}
vlib [file join $test_dir work_autoswing_handoff]
vmap -modelsimini $local_ini work_autoswing_handoff [file join $test_dir work_autoswing_handoff]
vlog -modelsimini $local_ini -work work_autoswing_handoff -sv \
    [file join $project_dir src pendulum_controller.v] \
    [file join $test_dir tb_autoswing_handoff.sv]
vsim -modelsimini $local_ini -voptargs=+acc work_autoswing_handoff.tb_autoswing_handoff
onfinish stop
run -all
quit -force -code 0
