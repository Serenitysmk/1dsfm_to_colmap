# 1dsfm_to_colmap
This tiny program can convert 1DSfM dataset into COLMAP database

How to use
----------

- COLMAP is required by CMake

- To convert a 1DSfM dataset into COLAMP, you just need to:
- Specify the dataset directory which contains the following files: `EGs.txt`, `list.txt`, `tracks.txt`, `coords.txt`
- Specify the directory which contains all the images
- Specify the output database path
- Compile:
```
cmake ..
make -j16
```
and 
- run:

```
./read_1dsfm --dataset_directory [dataset_directory] --image_path [image_path] --database_path [database_path]
```

Reference
---------

SfM_Init: https://github.com/wilsonkl/SfM_Init 

where the original file formats are described


