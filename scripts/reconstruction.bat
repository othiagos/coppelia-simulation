@echo off
set SCRIPT_PATH=%~dp0
set PATH=%SCRIPT_PATH%\lib;%PATH%
set QT_PLUGIN_PATH=%SCRIPT_PATH%\lib\plugins;%QT_PLUGIN_PATH%
set COLMAP=%SCRIPT_PATH%\bin\colmap

set WORKSPACE_PATH=%1
set IMAGES_PATH=%WORKSPACE_PATH%\%2
set MAX_IMAGE_SIZE=1600
set IS_MULTPLE_MODELS=1

if not exist %WORKSPACE_PATH% (
    echo "Invalid workspace folder"
    Exit /b
)

if not exist %IMAGES_PATH% (
    echo "Invalid image folder"
    Exit /b
)

if "%3%" == "A" (
    goto automatic_reconstructor
) else (
    goto reconstructor
)

:automatic_reconstructor
@REM {low, medium, high, extreme}
@echo Automatic Reconstructor
%COLMAP% automatic_reconstructor ^
    --workspace_path %WORKSPACE_PATH% ^
    --image_path %IMAGES_PATH% ^
    --quality medium ^
    --camera_model PINHOLE
goto save_statistics

:reconstructor
@echo Extract features
%COLMAP% feature_extractor ^
    --database_path %WORKSPACE_PATH%\database.db ^
    --image_path %IMAGES_PATH% ^
    --ImageReader.camera_model PINHOLE ^
    --SiftExtraction.max_image_size %MAX_IMAGE_SIZE% ^
    --SiftExtraction.max_num_features 4096

@echo Perform exhaustive matching
%COLMAP% exhaustive_matcher ^
    --database_path %WORKSPACE_PATH%\database.db

@echo Create sparse folder
mkdir %WORKSPACE_PATH%\sparse

@echo Run the mapper
%COLMAP% mapper ^
    --database_path %WORKSPACE_PATH%\database.db ^
    --image_path %IMAGES_PATH% ^
    --output_path %WORKSPACE_PATH%\sparse ^
    --Mapper.ba_local_max_num_iterations 25 ^
    --Mapper.ba_global_images_ratio 1.1000000000000001 ^
    --Mapper.ba_global_points_ratio 1.1000000000000001 ^
    --Mapper.ba_global_max_num_iterations 50 ^
    --Mapper.ba_global_max_refinements 5 ^
    --Mapper.multiple_models %IS_MULTPLE_MODELS%

@echo Create dense folder
mkdir %WORKSPACE_PATH%\dense

for /d %%A in (%WORKSPACE_PATH%\sparse\*) do (
    mkdir %WORKSPACE_PATH%\dense\%%~nxA
    
    @echo Undistort images
    %COLMAP% image_undistorter ^
        --image_path %IMAGES_PATH% ^
        --input_path %WORKSPACE_PATH%\sparse\%%~nxA ^
        --output_path %WORKSPACE_PATH%\dense\%%~nxA ^
        --output_type COLMAP ^
        --max_image_size %MAX_IMAGE_SIZE%

    @echo Perform stereo matching
    %COLMAP% patch_match_stereo ^
        --workspace_path %WORKSPACE_PATH%\dense\%%~nxA ^
        --workspace_format COLMAP ^
        --PatchMatchStereo.max_image_size %MAX_IMAGE_SIZE% ^
        --PatchMatchStereo.window_radius 5 ^
        --PatchMatchStereo.window_step 1 ^
        --PatchMatchStereo.num_samples 15 ^
        --PatchMatchStereo.geom_consistency 1

    @echo Perform stereo fusion
    %COLMAP% stereo_fusion ^
        --workspace_path %WORKSPACE_PATH%\dense\%%~nxA ^
        --workspace_format COLMAP ^
        --input_type geometric ^
        --output_path %WORKSPACE_PATH%\dense\%%~nxA\fused.ply ^
        --StereoFusion.max_image_size %MAX_IMAGE_SIZE% ^
        --StereoFusion.check_num_images 50

    @echo Generate mesh using Poisson meshing
    %COLMAP% poisson_mesher ^
        --input_path %WORKSPACE_PATH%\dense\%%~nxA\fused.ply ^
        --output_path %WORKSPACE_PATH%\dense\%%~nxA\meshed-poisson.ply
)

:save_statistics
echo "save_statistics"
@echo Save statistics about reconstructions
for /d %%A in (%WORKSPACE_PATH%\sparse\*) do (
    %COLMAP% model_analyzer ^
        --path %%A ^
        > %%A\stats.txt 2>&1

    %COLMAP% model_converter ^
        --input_path %%A ^
        --output_path %%A\model.nvm ^
        --output_type NVM
)
