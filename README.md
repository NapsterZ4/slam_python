# Instrucciones de ejecución


1. Crear un entorno y activarlo (venv) (opcional)

``
virtualenv venv
``

``
source venv/bin/activate
``

2. Ejecutar el archivo requirements.txt con las dependencias

``
pip install -r requirements.txt
``

3. Instalar el algoritmo de BreezySLAM
<pre>
git clone https://github.com/simondlevy/BreezySLAM?tab=readme-ov-file
cd BreezySlam/python
pip install .
</pre>


4. Ejecutar el algoritmo (es necesario tener habilitado el API de CoppeliaSim)

``
python3 main.py
``