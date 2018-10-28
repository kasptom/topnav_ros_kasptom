from shutil import rmtree, copytree, copyfile, move
from os import listdir, mkdir, remove

aruco_ids = []
for image_file in listdir("markers/images"):
    print (image_file)
    aruco_id = image_file.split(".")[0]
    aruco_ids.append(aruco_id)

aruco_ids = sorted(aruco_ids)

rmtree("arucos_255", ignore_errors=True)
mkdir("arucos_255")


def replace_placeholder_with_aruco_id(template_file_path, file_path, placeholder, marker_id):
    replaced_lines = []
    with open(template_file_path) as template:
        for line in template:
            if placeholder in line:
                line = line.replace(placeholder, aruco_id)
            replaced_lines.append(line)

    with open(file_path, mode="w") as model_file:
        model_file.writelines(replaced_lines)

    new_file_path = file_path.replace(placeholder, marker_id) if placeholder in file_path else file_path

    if new_file_path != file_path:
        move(file_path, new_file_path)


def generate_aruco_model_files_from_the_template(placeholder, marker_id, marker_dir_relative_path):
    file_paths = [
        marker_dir_relative_path + "/materials/scripts/ar_{{id}}.material",
        marker_dir_relative_path + "/model.config",
        marker_dir_relative_path + "/model.sdf",
        marker_dir_relative_path + "/model-1_4.sdf"
    ]
    image_file_path = marker_dir_relative_path + "/materials/textures/056.png"

    for file_path in file_paths:
        template_file_path = file_path + ".BAK"

        copyfile(file_path, template_file_path)
        replace_placeholder_with_aruco_id(template_file_path, file_path, placeholder, marker_id)
        remove(template_file_path)

    remove(image_file_path)
    copyfile("markers/images/%03d.png" % int(marker_id),
             marker_dir_relative_path + "/materials/textures/%03d.png" % int(marker_id))


for aruco_id in aruco_ids:
    aruco_dir_path = "arucos_255/ar_%03d" % int(aruco_id)
    copytree("ar_{{id}}", aruco_dir_path)
    generate_aruco_model_files_from_the_template("{{id}}", aruco_id, aruco_dir_path)
    print (aruco_id)
