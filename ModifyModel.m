
close all; clear all; clc; beep off;
%%
import org.opensim.modeling.*;
modelFile = "MOBL_ARMS_41_scaled.osim";
model = Model(modelFile);
muscles = model.updMuscles();

for i = 1:muscles.getSize()
    M = muscles.get(i-1);
    M.setMaxIsometricForce(2*M.getMaxIsometricForce());
end

model.finalizeConnections();
model.print('MOBL_ARMS_41_scaled_x2F.osim');