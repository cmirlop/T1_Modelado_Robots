# T1_Modelado_Robots

Proyecto Final :
- Descomprimir los 3 archivos .ZIP 
  - Uno es el entorno de Unity para hacer el test --> Proyecto_Final_Test_LIDAR_RL.zip
  - Otro es el entorno de Unity para entrenar el modelo de RL --> Proyecto_Final_EntrenoRL.zip
    - Hay 16 agentes, pero se puede utilizar solo con 8 modificando una variable del codigo python  
  - El otro es donde se almacenan los codigo de ros empleados, hay muchos pero los que necesitamos son:
    - cd /ros1/scr/ROS-TCP-Endpoint/scr/ros_tcp_endpoint/
    - python3 proyecto_basico.py --> Ejecuta el codigo en los 4 primeros agentes donde se prueba LIDAR y Camara
    - reinforce_Keras_4.py --> EN este codigo esta el modelo donde se encuentran las recompensas que hemos utilizado, y todo el control del robot, no hay que ejecutarlo
    - python3 train_ppo_maze.py --> Con este códgio ejecutamos en entrenamiento de 8 agentes en Unity para realizar el entrenamiento, aqui es donde se encuentras el modelo de la red neuronal, y los hiperparámetros que hemos tocado, como son los pasos, el batch, learning rate
      - Linea 10 --> Numero de entornos para entrenar
      - Linea 106 --> Esta cada cuantos pasos de cada robot se guardara una copia de seguridad , si hay 8 robots, cada 4000 pasos
      - Linea 112 --> Configuracion del early stop
      - Linea 125 --> Pasos totales que queremos que se hagan en el entrenamiento
    - python3 test_ckeckpoints.py --> Este codigo dentro de el podemos hacer que pruebe alguna copia de seguridad que ha almacenado, o el modelo en general solo comentando la linea 22
      - Linea 22 --> Se modifica la ruta donde estan los checkpoints, o la del modelo general, solo comentando y descomentando una linea o la superior ya funciona   
