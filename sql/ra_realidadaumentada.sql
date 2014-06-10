-- phpMyAdmin SQL Dump
-- version 3.5.2.2
-- http://www.phpmyadmin.net
--
-- Host: 127.0.0.1
-- Generation Time: May 13, 2014 at 03:03 AM
-- Server version: 5.5.27
-- PHP Version: 5.4.7

SET SQL_MODE="NO_AUTO_VALUE_ON_ZERO";
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;

--
-- Database: `ra_realidadaumentada`
--

-- --------------------------------------------------------

--
-- Table structure for table `imagen`
--

CREATE TABLE IF NOT EXISTS `imagen` (
  `id_imagen` int(11) NOT NULL AUTO_INCREMENT,
  `id_lugar` int(11) DEFAULT NULL,
  `nombre` varchar(250) DEFAULT NULL COMMENT 'indica el nombre en particular al que hace referencia esta imagen, este nombre es diferente al nombre del lugar donde se ubica',
  `ext` varchar(10) DEFAULT NULL COMMENT 'extension del archivo',
  `col_hist` varchar(400) DEFAULT NULL COMMENT 'guarda el vector correspondiente al histograma de color',
  `key_point` longtext COMMENT 'guarda el vector con los puntos caracteristicos sift',
  `id_image_col_hist` int(11) DEFAULT NULL COMMENT 'guarda el id de la imagen que es el vecino mas cercano usando como vector el histograma de color',
  `id_image_key_point` int(11) DEFAULT NULL COMMENT 'guarda el id de la imagen mas cercana (en el grupo de imagenes cercanas en el histograma) usando como vector los key points',
  `dst_col_hist` float DEFAULT NULL COMMENT 'guarda la distancia entre el vecino mas cercano y esta imagen, usando el histograma de color como vector',
  `dst_key_point` float DEFAULT NULL COMMENT 'guarda la distancia entre el vecino mas cercano y esta imagen, usando como vector los keypoints',
  `gps_lat` float DEFAULT NULL COMMENT 'indica la latitud',
  `gps_lng` int(11) DEFAULT NULL COMMENT 'indica la longitud',
  `width` int(11) DEFAULT NULL COMMENT 'ancho de la imagen',
  `height` int(11) DEFAULT NULL COMMENT 'alto de la imagen',
  `col_hist_maxFactor` int(11) DEFAULT NULL COMMENT 'Guardar el factor por el cual el histograma se normalizo',
  `dst_origin` float DEFAULT NULL,
  `creacion` timestamp NULL DEFAULT CURRENT_TIMESTAMP COMMENT 'fecha de creacion',
  PRIMARY KEY (`id_imagen`)
) ENGINE=InnoDB  DEFAULT CHARSET=latin1 AUTO_INCREMENT=4 ;

--
-- Dumping data for table `imagen`
--

INSERT INTO `imagen` (`id_imagen`, `id_lugar`, `nombre`, `ext`, `col_hist`, `key_point`, `id_image_col_hist`, `id_image_key_point`, `dst_col_hist`, `dst_key_point`, `gps_lat`, `gps_lng`, `width`, `height`, `col_hist_maxFactor`, `dst_origin`, `creacion`) VALUES
(1, NULL, 'cache/temporal_1399941422.png', NULL, ' 0 0.1602363667322 5.4177050601365E-5 1 0.00096744733216724 0.0023102642292154 0.56501246072164 0.040609569215052 0.6558364162655 0.17258486447998 4.2567682615358E-5', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 258412, 1.34405, '2014-05-13 00:37:04'),
(2, NULL, 'cache/temporal_1399941466.png', NULL, ' 0 0 1 0.055059312721907 0.11499871910774 0.0081426638748155 1.0585886472719E-5 0.12043986475471 0.34411752874597 0.021859855566165 0', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 472327, 1.07225, '2014-05-13 00:37:48'),
(3, NULL, 'cache/temporal_1399941592.png', NULL, ' 0 0.1602363667322 5.4177050601365E-5 1 0.00096744733216724 0.0023102642292154 0.56501246072164 0.040609569215052 0.6558364162655 0.17258486447998 4.2567682615358E-5', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 258412, 1.34405, '2014-05-13 00:39:54');

-- --------------------------------------------------------

--
-- Table structure for table `lugar`
--

CREATE TABLE IF NOT EXISTS `lugar` (
  `id_lugar` int(11) NOT NULL AUTO_INCREMENT,
  `nombre` varchar(250) NOT NULL COMMENT 'indica el nombre con el cual se conoce a este lugar',
  `descripcion` text NOT NULL COMMENT 'es una breve descripción del lugar',
  `gps_lat` int(11) NOT NULL COMMENT 'indica la latitud',
  `gps_lng` int(11) NOT NULL COMMENT 'indica la longitud',
  `creacion` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`id_lugar`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1 ;

-- --------------------------------------------------------

--
-- Table structure for table `vecino`
--

CREATE TABLE IF NOT EXISTS `vecino` (
  `id_imagen_src` int(11) NOT NULL COMMENT 'id de la imagen fuente',
  `id_imagen_dst` int(11) NOT NULL COMMENT 'id de la imagen destino',
  `dst` float NOT NULL COMMENT 'distancia entre las imagenes'
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
