package DroneFlightControl
  package Motor
    annotation(
      Icon(coordinateSystem(grid = {1, 1})));
  end Motor;

  package FlightControl
    model Controller
      // constants
      import Modelica.Constants.inf;
      import Modelica.Math.exp;
      parameter Modelica.Units.SI.Velocity flightVelocity = 22 "m/s";
      // parameters
      parameter Modelica.Units.SI.Height initialHeight = 0 "Initial ground height";
      // variables
      Modelica.Units.SI.Height currHeightProfile(start = initialHeight) "Current flight height";
      Modelica.Units.SI.Distance currDistance "Current flight distance";
      //connectors
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-25, -20}, {-5, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1, a(fixed = true, start = 0), phi(fixed = true, start = 0)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{2, -20}, {22, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.SpringDamper spring(c = 1e4, d = 100, stateSelect = StateSelect.prefer, w_rel(fixed = true)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{32, -20}, {52, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{22, -50}, {2, -30}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantTorque loadTorque(tau_constant = 10, useSupport = false) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{98, -15}, {88, -5}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia2(J = 2) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{60, -20}, {80, 0}}, rotation = 0)));
      Modelica.Blocks.Continuous.LimPID PI(Ni = 0.1, Td = 0.1, Ti = 0.1, controllerType = Modelica.Blocks.Types.SimpleController.PI, initType =               Modelica.Blocks.Types.Init.SteadyState, k = 100, limiter(u(start = 0)), yMax = 12) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-56, -20}, {-36, 0}}, rotation = 0)));
    Modelica.Blocks.Sources.RealExpression realExpression(y = currHeightProfile)  annotation(
        Placement(visible = true, transformation(origin = {-70, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      currDistance = flightVelocity*time;
      currHeightProfile = (2.065361*exp(-(((currDistance/flightVelocity) - 70.00864)^2)/(2*(22.97585^2))));
      connect(PI.y, torque.tau) annotation(
        Line(points = {{-35, -10}, {-27, -10}}, color = {0, 0, 127}));
      connect(torque.flange, inertia1.flange_a) annotation(
        Line(points = {{-5, -10}, {2, -10}}));
      connect(inertia1.flange_b, spring.flange_a) annotation(
        Line(points = {{22, -10}, {32, -10}}));
      connect(spring.flange_b, inertia2.flange_a) annotation(
        Line(points = {{52, -10}, {60, -10}}));
      connect(speedSensor.flange, inertia1.flange_b) annotation(
        Line(points = {{22, -40}, {22, -10}}));
      connect(loadTorque.flange, inertia2.flange_b) annotation(
        Line(points = {{88, -10}, {80, -10}}));
      connect(speedSensor.w, PI.u_m) annotation(
        Line(points = {{1, -40}, {-46, -40}, {-46, -22}}, color = {0, 0, 127}));
  connect(realExpression.y, PI.u_s) annotation(
        Line(points = {{-58, 28}, {-56, 28}, {-56, 14}, {-80, 14}, {-80, -10}, {-58, -10}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(grid = {1, 1})),
        experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.04),
        Diagram(graphics = {Rectangle(lineColor = {255, 0, 0}, extent = {{-99, 48}, {-32, 8}}), Rectangle(lineColor = {255, 0, 0}, extent = {{-25, 6}, {99, -50}}), Text(textColor = {255, 0, 0}, extent = {{4, 14}, {71, 7}}, textString = "plant (simple drive train)"), Text(textColor = {255, 0, 0}, extent = {{-98, -46}, {-60, -52}}, textString = "PI controller"), Text(textColor = {255, 0, 0}, extent = {{-98, 59}, {-31, 51}}, textString = "reference speed generation"), Line(points = {{-76, -44}, {-57, -23}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end Controller;

    model ground
      // constants
      import Modelica.Constants.inf;
      import Modelica.Math.exp;
      parameter Modelica.Units.SI.Velocity flightVelocity = 22 "m/s";
      // parameters
      parameter Modelica.Units.SI.Height initialHeight = 0 "Initial ground height";
      // variables
      Modelica.Units.SI.Height currHeightProfile(start = initialHeight) "Current flight height";
      Modelica.Units.SI.Distance currDistance "Current flight distance";
      //connectors
      heightConnect hProfile annotation(
        Placement(visible = true, transformation(origin = {4, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {3, 19}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      currDistance = flightVelocity*time;
      currHeightProfile = (2.065361*exp(-(((currDistance/flightVelocity) - 70.00864)^2)/(2*(22.97585^2))));
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Line(origin = {-24.88, -7.4}, points = {{-55.1177, -6.59965}, {-37.1177, 21.4003}, {-25.1177, 5.40035}, {-5.11768, 15.4003}, {4.88232, -0.599651}, {24.8823, 27.4003}, {26.8823, 19.4003}, {30.8823, 23.4003}, {56.8823, -8.59965}, {-57.1177, -8.59965}, {-61.1177, -8.59965}}, color = {0, 170, 0}, thickness = 5)}),
        experiment(StartTime = 0, StopTime = 150, Tolerance = 1e-06, Interval = 0.03),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {68, -16}, lineColor = {255, 0, 0}, extent = {{-99, 48}, {-32, 8}}), Text(origin = {68, -16}, textColor = {255, 0, 0}, extent = {{-98, 59}, {-31, 51}}, textString = "reference speed generation")}));
    end ground;

    model pid_control
      Modelica.Blocks.Continuous.LimPID pid annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DroneFlightControl.FlightControl.heightConnect hProfileIn annotation(
        Placement(visible = true, transformation(origin = {-82, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-61, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      rpm rpmOut annotation(
        Placement(visible = true, transformation(origin = {60, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {24, -26}, extent = {{22, -50}, {2, -30}}, rotation = 0)));
    equation
      connect(rpmOut, speedSensor.flange) annotation(
        Line(points = {{60, -2}, {86, -2}, {86, -66}, {46, -66}}));
      connect(speedSensor.w, pid.u_m) annotation(
        Line(points = {{26, -66}, {0, -66}, {0, -12}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}})));
    end pid_control;

    connector rpm
      Modelica.Units.SI.Length s;
      flow Real omega(unit = "rpm") "revolutions per minute value";
      annotation(
        Icon(coordinateSystem(grid = {1, 1})));
    end rpm;

    connector heightConnect
      Modelica.Units.SI.Length s;
      flow Real currHeightProfile;
      annotation(
        Icon(coordinateSystem(grid = {1, 1})));
    end heightConnect;

    model PID_Ctrl
      Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-63, 20}, {-43, 40}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.SpringDamper spring(c = 1e4, d = 100, stateSelect = StateSelect.prefer, w_rel(fixed = true)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{32, -20}, {52, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia2(J = 2) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{60, -20}, {80, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{22, -50}, {2, -30}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.ConstantTorque loadTorque(tau_constant = 10, useSupport = false) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{98, -15}, {88, -5}}, rotation = 0)));
      Modelica.Blocks.Continuous.LimPID PI(Ni = 0.1, Td = 0.1, Ti = 0.1, controllerType = Modelica.Blocks.Types.SimpleController.PI, initType = Modelica.Blocks.Types.Init.SteadyState, k = 100, limiter(u(start = 0)), yMax = 12) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-56, -20}, {-36, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1, a(fixed = true, start = 0), phi(fixed = true, start = 0)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{2, -20}, {22, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-25, -20}, {-5, 0}}, rotation = 0)));
      DroneFlightControl.FlightControl.heightConnect heightConnect annotation(
        Placement(visible = true, transformation(origin = {-86, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-82, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Units.SI.AngularVelocity u;
    equation
      connect(speedSensor.w, PI.u_m) annotation(
        Line(points = {{1, -40}, {-46, -40}, {-46, -22}}, color = {0, 0, 127}));
      connect(inertia1.flange_b, spring.flange_a) annotation(
        Line(points = {{22, -10}, {32, -10}}));
      connect(loadTorque.flange, inertia2.flange_b) annotation(
        Line(points = {{88, -10}, {80, -10}}));
      connect(integrator.y, PI.u_s) annotation(
        Line(points = {{-42, 30}, {-37, 30}, {-37, 11}, {-67, 11}, {-67, -10}, {-58, -10}}, color = {0, 0, 127}));
      connect(speedSensor.flange, inertia1.flange_b) annotation(
        Line(points = {{22, -40}, {22, -10}}));
      connect(PI.y, torque.tau) annotation(
        Line(points = {{-35, -10}, {-27, -10}}, color = {0, 0, 127}));
      connect(torque.flange, inertia1.flange_a) annotation(
        Line(points = {{-5, -10}, {2, -10}}));
      connect(spring.flange_b, inertia2.flange_a) annotation(
        Line(points = {{52, -10}, {60, -10}}));
      connect(heightConnect, integrator.u) annotation(
        Line(points = {{-86, 30}, {-64, 30}}));
      annotation(
        Icon(coordinateSystem(grid = {1, 1})),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {255, 0, 0}, extent = {{-25, 6}, {99, -50}}), Rectangle(lineColor = {255, 0, 0}, extent = {{-99, 48}, {-32, 8}}), Text(textColor = {255, 0, 0}, extent = {{-98, -46}, {-60, -52}}, textString = "PI controller"), Text(textColor = {255, 0, 0}, extent = {{4, 14}, {71, 7}}, textString = "plant (simple drive train)"), Line(points = {{-76, -44}, {-57, -23}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}), Text(textColor = {255, 0, 0}, extent = {{-98, 59}, {-31, 51}}, textString = "reference speed generation")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
    end PID_Ctrl;
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
