package DroneFlightControl
  package Motor
    annotation(
      Icon(coordinateSystem(grid = {1, 1})));
  end Motor;

  package FlightControl
  model Controller
    equation

      annotation(
        Icon(coordinateSystem(grid = {1, 1})));
    end Controller;

    class Ground
      //constants
      import Modelica.Constants.inf;
      //parameter
      //variables
      Modelica.Units.SI.Height height;
      Modelica.Units.SI.Distance distance;
      Modelica.Units.SI.Velocity velocity;
      
    equation
      der(distance) = velocity;
      der(velocity) = acceleration;
      
      annotation(
        Icon(coordinateSystem(grid = {1, 1})));
    end Ground;

    class PID
   parameter  Real K = 30;
      parameter  Real Ti = 3;  
      parameter  Real Td = 1;
      RealInput  e; 
      RealOutput y; 
      Real eprim;
      Real eint;
    equation 
      der(eprim) = e/Ti;
      eint = Td*der(e);
      y = K*(e + eprim + eint);
      annotation(
        Icon(coordinateSystem(grid = {1, 1})),
  experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.01));
    end PID;
    annotation(
      Icon(coordinateSystem(grid = {1, 1})));
  end FlightControl;

  package Propeller_Env
    annotation(
      Icon(coordinateSystem(grid = {1, 1})));
  end Propeller_Env;
  annotation(
    Icon(coordinateSystem(grid = {1, 1})));
end DroneFlightControl;
