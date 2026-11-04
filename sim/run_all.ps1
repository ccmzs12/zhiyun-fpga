param(
    [string]$ModelsimBin = $env:MODELSIM_BIN
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

if (-not $ModelsimBin) {
    $candidates = @(
        'D:\modelssim\win64',
        'C:\modeltech64_2020.4\win64',
        'C:\intelFPGA\20.1\modelsim_ase\win32aloem'
    )
    foreach ($candidate in $candidates) {
        if (Test-Path (Join-Path $candidate 'vlog.exe')) {
            $ModelsimBin = $candidate
            break
        }
    }
}

if (-not $ModelsimBin -or -not (Test-Path (Join-Path $ModelsimBin 'vlog.exe'))) {
    throw 'ModelSim was not found. Set MODELSIM_BIN to the directory containing vlib.exe, vlog.exe and vsim.exe.'
}

$vlib = Join-Path $ModelsimBin 'vlib.exe'
$vlog = Join-Path $ModelsimBin 'vlog.exe'
$vsim = Join-Path $ModelsimBin 'vsim.exe'
$workName = 'work_release'
$work = Join-Path $root $workName

if (Test-Path -LiteralPath $work) {
    Remove-Item -LiteralPath $work -Recurse -Force
}

& $vlib $workName

$sources = @(
    'rtl/fm_voice.v',
    'rtl/voice_engine.v',
    'rtl/mixer.v',
    'rtl/i2c_master_multi.v',
    'rtl/mpu6050_ctrl.v',
    'rtl/sensor_map.v',
    'rtl/hcsr04.v',
    'rtl/pitch_trend.v',
    'rtl/ssd1306_spi.v',
    'rtl/pitch_scope.v',
    'rtl/keypad_scan.v',
    'rtl/poly_key.v',
    'tb/voice_engine_ref.v',
    'tb/tb_engine.v',
    'tb/tb_tone.v',
    'tb/tb_i2c_multi.v',
    'tb/tb_mpu6050_ctrl.v',
    'tb/tb_sensor_map.v',
    'tb/tb_vib.v',
    'tb/tb_hcsr04.v',
    'tb/tb_pitch_trend.v',
    'tb/tb_pitch_scope.v',
    'tb/tb_timbre_compare.v',
    'tb/tb_twokey_5ms.v'
)

& $vlog -work $workName @sources

$tests = @(
    'tb_tone',
    'tb_engine',
    'tb_i2c_multi',
    'tb_mpu6050_ctrl',
    'tb_sensor_map',
    'tb_vib',
    'tb_hcsr04',
    'tb_pitch_trend',
    'tb_pitch_scope',
    'tb_timbre_compare',
    'tb_twokey_5ms'
)

foreach ($test in $tests) {
    Write-Host "=== $test ==="
    & $vsim -c -work $workName -do "run -all; quit -f" $test
}
