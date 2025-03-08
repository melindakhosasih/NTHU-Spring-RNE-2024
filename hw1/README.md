# Installation
- conda
    ```
    conda env create -f environment.yaml
    conda activate rne_hw1
    ```
- pip
    ```
    pip install -r requirements.txt
    ```

# How to Run
```
python navigation.py -s [basic/diff_drive/bicycle] -c [pid/pure_pursuit/stanley/lqr] -p [a_star/rrt/rrt_star] -m [map path]
# example
python navigation.py -s basic -c pid -p a_star -m Maps/map1.png
```