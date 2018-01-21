function post_install(var)

% This function copies the documentation (/docs directory) from the zip file to the octave package installation directory.
% This function is necessary because the OpenVD GitHub Doc page is hosted at /docs and Octave pkg function installs the documentation only from /doc.

currentdir = pwd;
pkgdir = var.dir;

mkdir(pkgdir);

cd(pkgdir)
cd ..

unzip('openvd-master.zip');
<<<<<<< HEAD
movefile('openvd-master/docs','openvd-0.0.0/');
rmdir('openvd-master','s');
=======
movefile('openvd/docs','openvd-0.0.0/');
rmdir('openvd','s');
>>>>>>> e2818c78712b61a8f768eee77a89e05cec5b9c9d
delete('openvd-master.zip')

cd(currentdir)

end
