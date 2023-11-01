package DroneFlightControl
  package Motor
    annotation(
      Icon(coordinateSystem(grid = {1, 1})));
  end Motor;

  package FlightControl
    model Controller
  DroneFlightControl.FlightControl.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-44, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  DroneFlightControl.FlightControl.PID pid annotation(
        Placement(visible = true, transformation(origin = {22, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  DroneFlightControl.FlightControl.height height annotation(
        Placement(visible = true, transformation(origin = {-10, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-13, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation

      annotation(
        Icon(coordinateSystem(grid = {1, 1})));
    end Controller;

    class Ground
      // constants
      import Modelica.Constants.inf;
      import Modelica.Math.exp;
      import Modelica.Math.e;
      // parameters
      parameter Modelica.Units.SI.Height initialHeight = 0 "Initial ground height";
      // variables
      Modelica.Units.SI.Height height(start = initialHeight) "Current flight height";
      Modelica.Units.SI.Distance distance "Current flight distance";
    equation
      height = 2.065361*exp^(-(distance - 70.00864)^2/(2*22.97585^2));
      annotation(
        Icon(coordinateSystem(grid = {1, 1})));
    end Ground;

    class PID
  Modelica.Blocks.Continuous.PID PID(k = 1.0, Ti = 0.1, Td = 0.01);
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})),
        experiment(StopTime = 40));
    end PID;

    connector height
      Real        currHeight;  // potential variable
      annotation(
        Icon(coordinateSystem(grid = {1, 1})));
    end height;
    annotation(
      Icon(coordinateSystem(grid = {1, 1})));
  end FlightControl;

  package Propeller_Env
    annotation(
      Icon(coordinateSystem(grid = {1, 1})));
  end Propeller_Env;
  annotation(
    Icon(coordinateSystem(grid = {1, 1})),
    uses(Modelica(version = "4.0.0")));
end DroneFlightControl;
