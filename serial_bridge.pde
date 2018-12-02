import processing.serial.*;

static class SerialBridge {

  static Serial myPort;      // The serial port
  static final int nb_data_per_actuator = 3;
  static final int num_data = NB_ACTUATORS * nb_data_per_actuator;
  static final int NEW_LINE = 10;
  static final int CHUNKSIZE = 62;
  static boolean goSendData = true;
  static boolean firstContact = true;
  static int splitIndex = 0;
  static String[] splittedDataOut;

  static String[] chunkData(String datain, int chunksize) {
    String[] result = new String[datain.length()/chunksize + (datain.length()%chunksize !=0 ? 1:0)];
    for (int i = 0; i<result.length-1; i++) {
      result[i] = datain.subSequence(i*chunksize, (i+1)*chunksize).toString();
    }
    result[result.length-1] = datain.subSequence((result.length-1)*chunksize, datain.length()).toString();
    return result;
  }

  static void serialCallBack(CableBot abot,Serial aport) {
    String rawdataIn = aport.readString();
    if (rawdataIn.length()>1) {
      String[] tokens = splitTokens(rawdataIn, ",");
      abot.receiveFrame(tokens);
      splitIndex = 0;
      aport.clear();
    } else {
      if (splitIndex ==0) {
        splittedDataOut = chunkData(abot.buildFrame(), CHUNKSIZE);
      }
      String chunktosend = splittedDataOut[splitIndex];
      if (splitIndex!=splittedDataOut.length-1)
        chunktosend+= 'A';
      else
        chunktosend+='\n'; 
      myPort.write(chunktosend);
      splitIndex+=1;
    }
  }
}
