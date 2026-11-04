set project_dir [file normalize [file join [file dirname [info script]] ..]]
set project_file [file join $project_dir ACG525_Pendulum_Control.gprj]

open_project $project_file
set_option -top_module acg525_pendulum_top
set_option -verilog_std v2001
run all
