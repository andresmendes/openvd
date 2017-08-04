function post_install(var)

% This function copies the documentation from the zip file to the octave package installation directory.
% This function is necessary because the OpenVD GitHub Doc page is hosted at /docs and Octave pkg function installs the documentation only from /docs

currentdir = pwd;
pkgdir = var.dir;

mkdir(pkgdir)

cd(pkgdir)
cd ..

unzip('openvd.zip');
movefile('openvd/docs','openvd-0.0.0/');
rmdir('openvd','s');

cd(currentdir)

end
