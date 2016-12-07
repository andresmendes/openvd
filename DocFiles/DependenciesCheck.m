

packageFilesList = getAllFiles('../+VehicleDynamicsLateral');
[packageFilesDep,packageProductDep] = matlab.codetools.requiredFilesAndProducts(packageFilesList);

packageDepNumber = length(packageProductDep);

% packageDepList =


for i = 1:packageDepNumber

    % packageDepList(i,:) = packageProductDep(i).Name;

end

examplesFilesList = getAllFiles('../Examples');
[examplesFilesDep,examplesProductDep] = matlab.codetools.requiredFilesAndProducts(examplesFilesList);

examplesDepNumber = length(examplesProductDep);

for i = 1:examplesDepNumber

    % examplesDepList(i,:) = examplesProductDep(i).Name;

end

disp(examplesDepList)
