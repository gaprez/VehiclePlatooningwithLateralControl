function [XRef, YRef,YawRef,VX,VY, time] = getPath(path,Ts,speed)

blk = 'PlatoonWithLateralControlExample/Unreal Simulation/Simulation 3D Scene Configuration';
switch path
    case 'straight'
        T = 50;
        [scenario, ~] = scenario_LF_01_Straight_RightLane(T,Ts,speed);
        set_param(blk, 'ProjectFormat', 'Default Scenes');
        set_param(blk, 'SceneDesc', 'Straight road');
        set_param(blk, 'ProjectName','C:\Program Files\MATLAB\R2021b\toolbox\shared\sim3d_projects\automotive_project\UE4\WindowsNoEditor\VehicleSimulation.exe');
        

    case 'curve'
        T = 80;
        [scenario, ~] = scenario_LF_04_Curve_RightLane(T,Ts,speed);
        set_param(blk, 'ProjectFormat', 'Default Scenes');
        set_param(blk, 'SceneDesc', 'Curved road');
        set_param(blk, 'ProjectName','C:\Program Files\MATLAB\R2021b\toolbox\shared\sim3d_projects\automotive_project\UE4\WindowsNoEditor\VehicleSimulation.exe');
        

    case 'highway'
        T = 250;
        [scenario, ~] = scenario_RRHighway_01_NoShadowToShadow(T,Ts,speed);
        set_param(blk, 'ProjectFormat', 'Unreal Executable');
        set_param(blk, 'ProjectName','C:\ProgramData\MATLAB\SupportPackages\R2021b\toolbox\shared\sim3dprojects\driving\RoadRunnerScenes\WindowsPackage\RRScene.exe');
        set_param(blk, 'ScenePath', '/Game/Maps/RRHighway');
end

time = (0:scenario.SampleTime:scenario.StopTime)';
[XRef, YRef,YawRef,VX,VY] = getReferencePathFromScenario(scenario,time);
end