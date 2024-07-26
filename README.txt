	
                        MoToFlex-Simulator
                        ******************

For setup on Linux see the file tools/automated_run_docker/Dockerfile
and the run.sh. 

You can start the training on the Lamarr DGX cluster:

MOTOFLEX_ROOT=<full_path_to>/MoToFlex WANDB_API_KEY=<YOUR_API_KEY> screen srun --mem=32GB --export ALL -c 16 --container-name=motoflex_0 -p GPU1 --gres=gpu:1 --container-image=nvcr.io/ml2r/fraunhofer/motoflex_docker:latest --container-env=WANDB_API_KEY --container-env=MOTOFLEX_ROOT --pty <full_path_to>/MoToFlex/tools/automated_run_docker/run.sh

Also note the Lamarr DGX cluster info website:

https://gitlab.tu-dortmund.de/lamarr/lamarr-public/cluster#vscode-remote-extension

                        Important Commits
                        ******************

Last commit with all observations and reward components:
https://github.com/OliverUrbann/MoToFlex/tree/7215843e0d93b07fa3cfe4d31ae10c9113c7a835

                          Obsolete Setup
                        ******************

A) Installation and Setup (Windows)

  1. Extract folder contents to any directory
  2. Run Premake4.bat to create a Visual Studio 2010 solution.
     If you want to work with Visual Studio 2008 change the
     parameter in Premake4.bat
  3. In folder "Make" open the solution and compile "Simulator".
     The other project is for the black box parameter optimization.
  4. In the project properties of "Simulator" set "Working Directory"
     under "Debugging" to "$(ProjectDir)../".
  5. Set "Command Arguments" to the config file you want to use.
     The simulation comes with three different configurations which 
     will be explained in B. For the first run set "config/200.cfg".
  6. Start "Simulator".

************************************************************************

B) Configurations

  There are currently two different ways to create the angles that are
  sent to the robot. First it is possible to use predefined angles. The
  configuration file 50.cfg configures a walk at 50 mm/s, 200.cfg a
  walk at 200 mm/s. The second possible way is to use the included
  "Dortmund Walking Engine", which can create the target angles online.
  To switch between the predefined angles and the walking engine do 
  the following:

  Switching to predefined angles:
  
  1. In eaconfig.h set #define SOURCE_DT 0.02
  2. In eaconfig.h uncomment 
     typedef AngleFromCSV AngleAdapter;
     and comment out any other typedef AngleAdapter.
  3. In the "Simulator" project properties set "Command Arguments"
     to a valid configuration (50.cfg or 200.cfg).

  Switching to walking engine:

  1. In eaconfig.h set #define SOURCE_DT 0.01
  2. In eaconfig.h uncomment
     typedef AngleFromCSV AngleAdapter;
     and comment out any other typedef AngleAdapter.
  3. In the "Simulator" project properties set "Command Arguments" to
     config/DortmundWalkingEngine.cfg
