figure('Name', 'Drone Data Visualization');
gcf
clf

set(groot,'defaultAxesFontName','Franklin Gothic Book')
set(groot,'defaultAxesFontSize',18)
subplot(3,3,1:3)

plot(exportedVariables.time, exportedVariables.defHeightProfileMeasHeightProfile, 'r', 'LineWidth', 2.5);
hold on;
plot(exportedVariables.time, exportedVariables.defSensorMeasHeightProfileOut, 'b', 'LineWidth', 3.5);
plot(exportedVariables.time, exportedVariables.defDroneDroneHeightOut, 'g', 'LineWidth', 2.5);
hold off;
title('Flug- und Höhenprofil');
ylabel('Höhe ab Start (m)');
legend('Höhenprofil Rocky Mountains', 'Soll-Höhe', 'Ist-Höhe');
grid on
grid minor

subplot(3,3,4:6)
plot(exportedVariables.time, exportedVariables.defControllerrpmRef, 'b', 'LineWidth', 3.5);
hold on;
plot(exportedVariables.time, exportedVariables.defPropeller0rpmProp, 'g', 'LineWidth', 2.5);
hold off;
title('RPM Regler-Sollvorgabe and Propeller RPM');
ylabel('Drehzahl (1/min)');
legend('Soll-RPM Regler', 'Ist-RPM Propeller');
grid on
grid minor

subplot(3,3,7:9)
plot(exportedVariables.time, exportedVariables.defDroneF_g, 'b', 'LineWidth',  3.5);
hold on;
plot(exportedVariables.time, exportedVariables.defDroneF_y, 'g', 'LineWidth', 2.5);
hold off;
title('Kräfte am Drohnenkörper');
ylabel('Kraft (N)');
xlabel('Zeit (s)');
legend('Gewichtskraft F_g', 'Auftriebskraft F_y', 'Location','northeast');
grid on
grid minor

% subplot(3,1,4)
% plot(exportedVariables.time, exportedVariables.defBatterycellStackSOC, 'b', 'LineWidth', 2.5);
% title('Batterie Cell Stack SOC');
% xlabel('Zeit (s)');
% ylabel('State of Charge (%)');
% grid on
% grid minor