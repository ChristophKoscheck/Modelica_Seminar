package DroneFlightControl
  package flight_control
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
      Modelica.Units.SI.AngularVelocity currReqRPM;
      //connectors
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-25, -20}, {-5, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1, a(fixed = true, start = 0), phi(fixed = true, start = 0)) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{2, -20}, {22, 0}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{22, -50}, {2, -30}}, rotation = 0)));
      Modelica.Blocks.Continuous.LimPID PI(Ni = 0.1, Td = 0.1, Ti = 0.1, controllerType = Modelica.Blocks.Types.SimpleController.PI, initType = Modelica.Blocks.Types.Init.SteadyState, k = 100, limiter(u(start = 0)), yMax = 12) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-56, -20}, {-36, 0}}, rotation = 0)));
      Modelica.Blocks.Sources.RealExpression realExpression(y = currHeightProfile) annotation(
        Placement(visible = true, transformation(origin = {-78, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain gain(k = 20/9.55) annotation(
        Placement(visible = true, transformation(origin = {-74, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput ctrlRefomega annotation(
        Placement(visible = true, transformation(origin = {64, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {58, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DroneFlightControl.flight_control.omegaAngularVel omegaAngularVel annotation(
        Placement(visible = true, transformation(origin = {64, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {78, -61}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      currDistance = flightVelocity*time;
      currHeightProfile = (2.065361*exp(-(((currDistance/flightVelocity) - 70.00864)^2)/(2*(22.97585^2))));
      currReqRPM = currHeightProfile^2;
      connect(PI.y, torque.tau) annotation(
        Line(points = {{-35, -10}, {-27, -10}}, color = {0, 0, 127}));
      connect(torque.flange, inertia1.flange_a) annotation(
        Line(points = {{-5, -10}, {2, -10}}));
      connect(speedSensor.flange, inertia1.flange_b) annotation(
        Line(points = {{22, -40}, {22, -10}}));
      connect(speedSensor.w, PI.u_m) annotation(
        Line(points = {{1, -40}, {-46, -40}, {-46, -22}}, color = {0, 0, 127}));
      connect(gain.y, PI.u_s) annotation(
        Line(points = {{-63, -10}, {-58, -10}}, color = {0, 0, 127}));
      connect(realExpression.y, gain.u) annotation(
        Line(points = {{-66, 32}, {-52, 32}, {-52, 18}, {-94, 18}, {-94, -10}, {-86, -10}}, color = {0, 0, 127}));
      connect(speedSensor.w, ctrlRefomega) annotation(
        Line(points = {{2, -40}, {-12, -40}, {-12, -64}, {40, -64}, {40, -10}, {64, -10}}, color = {0, 0, 127}));
      connect(speedSensor.w, omegaAngularVel) annotation(
        Line(points = {{2, -40}, {-18, -40}, {-18, -72}, {64, -72}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(grid = {1, 1})),
        experiment(StartTime = 0, StopTime = 200, Tolerance = 1e-06, Interval = 0.01),
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

    connector omegaAngularVel
      flow Real omega(unit = "rad/s");
      annotation(
        Icon(coordinateSystem(grid = {1, 1}, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {3, -7.5}, fillColor = {255, 85, 127}, fillPattern = FillPattern.Solid, extent = {{-55, 52.5}, {55, -52.5}})}));
    end omegaAngularVel;

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
  end flight_control;

  package e_motor
    model Gleichstrommotor
      //Parameter
      parameter Real J = 0.003 "Trägheitsmoment des Motors (kg*m^2)";
      parameter Real B = 0.5 "Viskoser Dämpfungskoeffizient (N*m*s/rad)";
      parameter Real K = 0.5 "Motor-Konstante (N*m/A)";
      parameter Real R = 1.0 "Wicklungswiderstand (Ohm)";
      parameter Real L = 0.1 "Wicklungsinduktivität (H)";
      parameter Real k_p = 0.5 "Proportionaler Regelungsfaktor";
      parameter Real k_i = 4 "Integraler Regelungsfaktor";
      // parameter Real desiredSpeed = 20000*2*3.14159/60 "Gewünschte Drehzahl (rad/s)";
      parameter Real maxCurrent = 3500.0 "Maximal zulässiger Strom (A)";
      //Variablen
      input Modelica.Blocks.Interfaces.RealInput desiredSpeed "Gewünschte Drehzahl (rad/s)" annotation(
        Placement(visible = true, transformation(origin = {-52, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-52, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      output Real u "Stromzufuhr (A)";
      Real i(fixed = false) "Strom durch die Wicklung (A)";
      Real tau "Motor-Drehmoment (N*m)";
      Real integral_error "Integralfehler (rad)";
      Real test;
      DroneFlightControl.flight_control.omegaAngularVel omegaAngularVel annotation(
        Placement(visible = true, transformation(origin = {-44, -44}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-48, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation(
        Placement(visible = true, transformation(origin = {50, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      test = omegaAngularVel.omega*10;
      tau = K*i;
      J*der(omegaOut) = tau - B*omegaOut;
      L*der(i) = min(maxCurrent, u) - R*i - K*flange_b.phi;
// Regelung auf die gewünschte Drehzahl
      u = k_p*(desiredSpeed - omegaOut) + k_i*integral_error;
// Berechnung des Integralfehlers
      der(integral_error) = desiredSpeed - flange_b.phi;
    initial equation
      flange_b.phi = 0;
      integral_error = 0;
      i = 0;
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.02));
    end Gleichstrommotor;
  end e_motor;

  package propeller_env
    class propeller
      //Parameter
      parameter Modelica.Units.SI.Diameter d = 0.3 "Diameter";
      parameter Modelica.Units.SI.Area A_p = 0.13 "Area of the propeller";
      parameter Real A_prop = 0.13 "Area of the propeller";
      parameter Real r_prop = 0.2 "Radius of the propeller";
      parameter Real d_prop = 0.4 "Diameter of the propeller";
      parameter Real C_l = 0.006 "Lift coefficient";
      parameter Real NoP = 4 "Number of propellers";
      parameter Real c_w = 0.14 "Drag Coefficient";
      annotation(
        Icon(graphics = {Line(origin = {6.05, -26.45}, points = {{-15, 24}, {-15, -24}, {15, -24}, {15, 22}, {-15, 22}, {-1, 22}}, thickness = 0.5), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {-42, -4}, extent = {{36, -6}, {-36, 6}}), Ellipse(origin = {56, -6}, extent = {{36, -6}, {-36, 6}})}));
    end propeller;

    model Auftrieb
      //Parameter
      parameter Real p_air = 1.225 "Air density";
      parameter Real A_prop = 0.13 "Area of the propeller";
      parameter Real r_prop = 0.2 "Radius of the propeller";
      parameter Real d_prop = 0.4 "Diameter of the propeller";
      parameter Real C_l = 0.006 "Lift coefficient";
      parameter Real NoP = 4 "Number of propellers";
      parameter Real initialSpeed = 1000 "Start RPM";
      parameter Real finalSpeed = 3000 "End RPM";
      parameter Real Masse = 4 "System Mass";
      //Variablen
      Real n_prop "Propeller speed";
      Real w_prop "Propeller angular velocity";
      Real F_auf "Lift force";
      Real F_prop "Propeller force";
      Real T_prop "Propeller torque";
      Real Acc_prop "acceleration per propeller";
      Real V_prop "Velocity of System";
    equation
      n_prop = if time <= 10 then initialSpeed + (finalSpeed - initialSpeed)*time/10 else finalSpeed;
      F_auf = NoP*F_prop;
//Force of all propellers
      F_prop = T_prop/r_prop;
//Force of one propeller
      T_prop = 0.5*p_air*A_prop*C_l*d_prop^2*w_prop^2;
//Torque of one propeller
      w_prop = 2*n_prop*Modelica.Constants.pi/60;
//Angular velocity of one propeller
      Acc_prop = F_prop/Masse;
//Acceleration of one propeller
      der(Acc_prop) = V_prop;
//Acceleration of the system
    end Auftrieb;

    model Auftrieb2
      //Parameter
      //Variablen
      Modelica.Units.SI.AngularVelocity w_prop "Propeller angular velocity";
      Modelica.Units.SI.Force F_auf "Lift force";
      Modelica.Units.SI.Force F_prop "Propeller force";
      Modelica.Units.SI.Torque T_prop "Propeller torque";
      Modelica.Units.SI.Acceleration Acc_prop "acceleration per propeller";
      Modelica.Units.SI.velocity V_prop "Velocity of System";
      Modelica.Units.SI.Resistance F_w "Flow resistance";
  //Input Variable
      // SI.Angle phi "Absolute rotation angle of flange";
      // flow SI.Torque tau "Cut torque in the flange";
      //Interface
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(
        Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
// omegaIn = if time <= 10 then initialSpeed + (finalSpeed - initialSpeed)*time/10 else finalSpeed;
      F_auf = propeller.NoP*F_prop "Force of all propellers";
      F_prop = T_prop/propeller.r_prop - F_w "Force of one propeller";
//T_prop  = 0.5*environment.p_air*prop.A_prop*propeller.C_l*propeller.d_prop^2*w_prop^2  "Torque of one propeller";
      T_prop = flange.tau "Torque of one propeller";
      w_prop = propeller.d_prop*flange.phi "Angular velocity of one propeller";
      Acc_prop = F_prop/drone.Masse "Acceleration of one propeller";
      der(V_prop) = Acc_prop "Acceleration of the system";
      F_w = drone.A_drone*propeller.c_w*0.5*environment.p_air*V_prop^2 "Flow resistance";
      annotation(
            Icon);
    end Auftrieb2;

    class environment
      parameter Modelica.Units.SI.Density p_air = 1.225 "Air density";
    end environment;

    class drone
      parameter Real initialSpeed = 1000 "Start RPM";
      parameter Real finalSpeed = 14000 "End RPM";
      parameter Modelica.Units.SI.Mass Masse = 4 "System Mass";
      parameter Modelica.Units.SI.Area A_drone = 0.5 "Dem wind ausgesetzte Drohnenoberfläche";
    end drone;
  end propeller_env;

  model DroneFlightControlFly
    DroneFlightControl.e_motor.Gleichstrommotor gleichstrommotor annotation(
      Placement(visible = true, transformation(origin = {-32, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    DroneFlightControl.flight_control.Controller controller annotation(
      Placement(visible = true, transformation(origin = {-78, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Fixed fixed annotation(
      Placement(visible = true, transformation(origin = {4, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  DroneFlightControl.propeller_env.Auftrieb2 auftrieb2 annotation(
      Placement(visible = true, transformation(origin = {34, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  connect(auftrieb2.flange_a, gleichstrommotor.flange_b) annotation(
      Line(points = {{24, 14}, {-22, 14}}));
    annotation(
      Icon(coordinateSystem(grid = {1, 1})),
      experiment(StartTime = 0, StopTime = 200, Tolerance = 1e-06, Interval = 0.01));
  end DroneFlightControlFly;

  connector Connector_2
  Modelica.Mechanics.Rotational.Interfaces.Flange flange annotation(
      Placement(visible = true, transformation(origin = {-42, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-42, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  annotation(
      Icon(graphics = {Rectangle(origin = {3, 5}, fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-43, 25}, {43, -25}})}));
  end Connector_2;
  annotation(
    Icon(coordinateSystem(grid = {1, 1})),
    uses(Modelica(version = "4.0.0")));
end DroneFlightControl;
