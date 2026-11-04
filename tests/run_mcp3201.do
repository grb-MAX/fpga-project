onerror {quit -force -code 1}
set project_dir [file normalize [pwd]]
set test_dir [file join $project_dir tests]
set local_ini [file join $project_dir modelsim.ini]
if {![file exists $local_ini]} {vmap -c}
vlib [file join $test_dir work]
vmap -modelsimini $local_ini work [file join $test_dir work]
vlog -modelsimini $local_ini -sv [file join $project_dir src mcp3201_spi.v] [file join $test_dir tb_mcp3201.sv]
vsim -modelsimini $local_ini -voptargs=+acc work.tb_mcp3201
onfinish stop
run -all
set passed [examine -radix unsigned /tb_mcp3201/test_passed]
if {$passed != 1} {quit -force -code 1}
quit -force -code 0
