set cfg "prj/impl/zhiyun_process_config.json"
file mkdir "prj/impl"
if {![file exists $cfg] && [file exists "prj/zhiyun_process_config.json"]} {
    file copy "prj/zhiyun_process_config.json" $cfg
}
if {[file exists $cfg]} {
    set fp [open $cfg r]
    set data [read $fp]
    close $fp
    set data [string map {"\"CPU\": false" "\"CPU\": true"} $data]
    set data [string map {"\"TopModule\": \"\"" "\"TopModule\": \"top\""} $data]
    set data [string map {"\"TopModule\": \"adsr\"" "\"TopModule\": \"top\""} $data]
    set fp [open $cfg w]
    puts -nonewline $fp $data
    close $fp
}

open_project {prj/zhiyun.gprj}
run all
