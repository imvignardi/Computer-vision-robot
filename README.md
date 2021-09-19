# Robot con visión por computador
Código relacionado con la interfaz gráfica de usuario y la visión por computador de Raspberry.

Los archivos en 05-sep, carpeta que se encuentra dentro de TFG, corresponden con la entrega realizada junto con la memoria. Las sucesivas versiones se guardan en carpetas diferentes nombradas mediante su version.

Para las pruebas se debe tener en cuenta lo siguiente.

  -Se requiere instalar opencv-contrib-python que contenga librerías para tracking (v4.4.0.46 válida).
	
  -Se necesita la librería imutils.
	
  -Se necesita la librería PyQt5
	
  -Se necesita la librería numpy la cual, en algunos casos, viene con el SO y otros lo instala opencv-contrib-python.
	
  -Dependiendo del sistema operativo instalado, la versión de Python y otras variables es posible que se requieran librerías adicionales que
   se deben instalar según pida el intérprete.
	 
  -Algunos casos requieren del cambio de la configuración de la biblioteca de componentes gráficos utilizada por PyQt5. 
	 Esto se puede modificar en /etc/xdg/qt5ct/qt5ct.conf cambiando style=gtk2 a style=gtk3 (Depende de la biblioteca que utiliza la máquina).
	 
   
  -Para comprobar el código de raspberry se puede utilizar cualquier ordenador, siempre y cuando se conozca el puerto en el que se 
   encuentra Arduino. Las pruebas en Windows suelen requerir la instalación de python3 y pip.
	 
  -Si no se dispone de Arduino las pruebas son complejas de realizar, aunque proporcionan más información. Se necesita conectar la Raspberry 
   mediante un conversor USB-UART a un ordenador y actuar en este mediante un terminal como RealTerm.
  
