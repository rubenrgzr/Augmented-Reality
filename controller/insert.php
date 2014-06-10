<?php

/*Este script recibe una imagen en formato base64*/


/*parametros configurables del script*/
$min_width = 700;
$min_height = 700;
$radio = 0.1;
$dirBase = "C:\\xampp\\htdocs\\ar\\";

$response;
/*se reemplaza la cadena que indica el formato de la imagen*/

	$data = str_replace("data:image/png;base64,","",$_POST["img"]);
	$data = base64_decode(str_replace("data:image/jpeg;base64,","",$data));

/*Se crea la imagen*/

	$im = imagecreatefromstring($data);
	$fileName = "";
	$fingerDir = "";
	if ($im !== false) {
		$date = new DateTime();
		header('Content-Type: image/png');
		$fileName = "cache/temporal_".$date->getTimestamp().".png";
		$fingerDir = "cache/temporal_".$date->getTimestamp().".pgm";
		imagepng($im,$fileName);
		imagedestroy($im);		
	}
	else {
		echo 'Ocurrió un error. La imagen enviada tiene un formato incorrecto.';
	}
	
	/*Validar la resolucion de la imagen, no debe ser menor a 800x800*/
	$sz = getimagesize($fileName);
	if($sz[0]<$min_width || $sz[1]<$min_height){
		$response["status"] = 1;
		$response["error"] = "El tamano de la imagen debe ser mayor a 800 de ancho y 800 de alto";
		echo json_encode($response);
	}
	 
	/*Ejecutar el archivo .exe para extraer el histograma de color*/
	$dir = $dirBase."controller/$fileName";
	$cubeDir =  $dirBase."controller/exe/nCortesCie1peso5delta64Mu0.010Nuevo";	
	exec($dirBase."controller\\exe\\ar_color_histogram  $dir $cubeDir  2000 2000 $fingerDir 2>&1",$out);
	
	/*tomar el histograma y normalizar para que los valores esten entre cero y uno*/
	
	$response["hist"] = explode(" ",$out[0]);
	$max = $response["hist"][0];
	for($i=0; $i< 11; $i=$i+1){
		if($max < $response["hist"][$i]){
			$max = $response["hist"][$i];
		}
	}
	$colHist="";
	if($max !=0 )
		for($i=0; $i< 11; $i=$i+1){		
			$response["hist"][$i]=$response["hist"][$i]/$max;		
			$colHist = $colHist." ".$response["hist"][$i];
		}
	/*calcular la norma del vector al punto hist*/
	$norm = 0;
	for($i=0; $i< 11; $i=$i+1){		
		$norm = $norm + $response["hist"][$i]*$response["hist"][$i];		
	}
	$norm = sqrt($norm);
	$response["norm"] = $norm;
	
	/*NOTA: la norma maxima puede ser de 11 cuando todos los elemtos son 1*/
	
	
	
	/*Conexion con la base de datos*/
	$link = mysql_connect('localhost', 'root', '');
	if (!$link) {
		die('Could not connect: ' . mysql_error());
	}
		
	/*conectar a la base de datos*/
	$conn = mysql_select_db('ra_realidadaumentada', $link);
	
	/*Hacer una consulta y buscar otro elemento con el mismo histograma y mismo factor de multiplicacion, puede ser muy pero muy posible que si existe, la imagen sea la misma*/ 
	$sql = 'SELECT count(*) as cnt from imagen WHERE col_hist = "'.$colHist.'" and col_hist_maxFactor = "'.$max.'"';
	$result = mysql_query($sql);
	
	if($result){
		$row = mysql_fetch_assoc($result);		
		$response["equal"] = $row["cnt"];
	}
	
	/*Si la imagen no "existe", insertarla a la base de datos*/
	if($response["equal"]==0){
		$sql = 'INSERT into imagen(nombre,col_hist,col_hist_maxFactor,dst_origin) VALUES ("'.$fileName.'","'.$colHist.'","'.$max.'","'.$norm.'")';
		$result = mysql_query($sql);
	}
	
	/*Buscar todas las imagenes que esten dentro de una banda*/
	
	$limitA = $radio + $norm;
	$limitB = -$radio + $norm;
	
	$sql = 'SELECT id_imagen,id_lugar,col_hist from imagen WHERE dst_origin >='.$limitB.' and dst_origin <= '.$limitA.' ORDER BY id_imagen';
	$result = mysql_query($sql);
	
	$idMin;
	$dstMin;
	$neighbors = array();
	if($result){
		while ($row = mysql_fetch_assoc($result)) {
			
			/*Calcular la distancia entre los 2 puntos*/
			$vec1 = explode(" ",$row["col_hist"]);
			$dst = 0;
			$aux=0;
			for($i=0; $i< 11; $i=$i+1){
				$aux = (   floatval(  $response["hist"][$i] ) - floatval( $vec1[$i] ) );
				$dst = $dst + $aux*$aux;
			}
			$dst = sqrt($dst);
			if($dst <= $radio){
				$row["dst"] = $dst;
//				array_push($neighbors,$row);
			}
			$row["dst"] = $dst;
			array_push($neighbors,$row);
		}
		$response["neighbors"]=$neighbors;
	}else{
		$response["neighbors"]=0;
	}
	
    echo json_encode($response);
?>