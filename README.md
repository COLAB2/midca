[![Documentation Status](https://readthedocs.org/projects/midca/badge/?version=latest)](https://midca.readthedocs.io/en/latest/?badge=latest)
# MIDCA: The Metacognitive Integrated Dual-Cycle Architecture.

MIDCA Version 1.4: User manual and tutorial for the Metacognitive Integrated Dual-Cycle Architecture (https://tinyurl.com/midcadoc)

0. (Recommended, but optional) Use a virtualenv before installing MIDCA:

   ```
   cd midca/
   python3 -m venv .env
   source .env/bin/activate
   ```

   Make sure to `source .env/bin/activate` whenever you open a new terminal to run MIDCA. If you are running MIDCA from PyCharm it should automatically detect the environment and use that version of python.


1. To install MIDCA, run the setup.py file: 

    ```
    python setup.py install
    ```
    
    If you will be making changes to MIDCA's code, use the `develop` option, like the following:

     ```
    python setup.py develop
    ```

3. For a simple interactive version of MIDCA, run

    ```
    cd midca/
    python examples/cogsci_demo.py
    ```

4. To see how the MIDCA instantiation used in `cogsci_demo.py` is created and populated, see `examples/cogsci_demo.py`

5. For an overview of MIDCA and more details about how it works, see the github wiki
   (https://github.com/COLAB2/midca/wiki) and/or docs folder.

6. Questions and comments are welcome, please email wsri-midca-help@wright.edu
