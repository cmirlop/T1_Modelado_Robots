# T1_Modelado_Robots

Proyecto Final :
- Descomprimir los 3 archivos .ZIP en un directorio
  - Uno es el entorno de Unity para hacer el test
  - Otro es el entorno de Unity para entrenar el modelo de RL
  - El otro es donde se almacenan los codigo de ros empleados, hay muchos pero los que necesitamos son:
    - cd /ros1/scr/ROS-TCP-Endpoint/scr/ros_tcp_endpoint/
    - python3 proyecto_basico.py --> Ejecuta el codigo en los 4 primeros agentes donde se prueba LIDAR y Camara
    - python3 reinforce_Keras_4.py --> EN este codigo esta el modelo donde se encuentran las recompensas que hemos utilizado, y todo el control del robot
    - python3 train_ppo_maze.py --> Con este códgio ejecutamos en entrenamiento de 8 agentes en Unity para realizar el entrenamiento, aqui es donde se encuentras el modelo de la red neuronal, y los hiperparámetros que hemos tocado, como son los pasos, el batch, learning rate
    - python3 test_ckeckpoints.py --> Este codigo dentro de el podemos hacer que pruebe alguna copia de seguridad que ha almacenado, o el modelo en general solo comentando la linea 22   
