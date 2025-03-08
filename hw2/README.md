# Installation
- conda
    ```
    conda env create -f environment.yaml
    conda activate rne_hw2
    ```
- pip
    ```
    pip install -r requirements.txt
    ```

# How to Run
- Train PPO
    ```
    python train.py
    ```
    - Plot learning curve
        ```
        python plot.py
        ```
- Evaluate
    ```
    python eval.py
    ```
- Demo
    ```
    python play.py
    ```