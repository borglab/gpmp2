import gtsam.*
import gpmp2.*

% Test parameterization 
arm = generateArm('KinovaGen3');

% Verify 
t = arm.fk_model();
assert(strcmp(t.parameterization,"MODIFIED_DH"),"Wrong parameterization returned");
assert(strcmp(t.parameterizationString,"Modified Denavit-Hartenberg"),"Wrong parameterization returned");