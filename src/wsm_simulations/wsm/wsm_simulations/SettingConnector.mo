within wsm_simulations;

connector SettingConnector
  output QTableSettingRecord r;
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, lineColor = {0, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Sphere, extent = {{-100, -100}, {100, 100}})}), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {0, 16.667}, textColor = {64, 64, 64}, extent = {{-160, 33.333}, {40, 73.333}}, textString = "%name"), Ellipse(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-40, -40}, {40, 40}})}));
end SettingConnector;
