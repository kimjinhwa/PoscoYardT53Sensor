#include <BluetoothSerial.h>
#include "bluetoothtask.h"
#include "WiFi.h"
#include "SimpleCLI.h"
#include "EEPROM.h"
#include "max14921.h"
#include "../../src/main.h"
#include "../../version.h"
SimpleCLI simpleCli;
BluetoothSerial SerialBT;

void readInputSerialBT();
void simpleCliSetup();
void reboot_Callback(cmd *c){
    SerialBT.printf("Rebooting...\n");
    ESP.restart();
}
void version_Callback(cmd *c){
    String ver = String(VERSION);
    SerialBT.printf("Ver: %s\n", ver.c_str());
}
void id_Callback(cmd *cmdPtr){
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String id = arg.getValue();
    if(id.length() > 0){
        SerialBT.printf("Current Modbus ID : %d\n", nvmSet.ModbusAddress);
        nvmSet.ModbusAddress = id.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("New Modbus ID : %d\n", nvmSet.ModbusAddress);
    }
    else{
        SerialBT.printf("Modbus ID : %d\n", nvmSet.ModbusAddress);
    }
}
void cells_Callback(cmd *cmdPtr){
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String cells = arg.getValue();
    if(cells.length() > 0){
        SerialBT.printf("Current Installed Cells : %d\n", nvmSet.InstalledCells);
        nvmSet.InstalledCells = cells.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(1, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("New Installed Cells : %d\n", nvmSet.InstalledCells);
    }
    else{
        SerialBT.printf("Installed Cells : %d\n", nvmSet.InstalledCells);
    }

}
void ssid_Callback(cmd *cmdPtr){
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String ssid = arg.getValue();
    if(ssid.length() > 0){
        if(ssid.compareTo("erase") == 0){
            strncpy(nvmSet.SSID, "", sizeof(nvmSet.SSID));
            strncpy(nvmSet.PASS, "", sizeof(nvmSet.PASS));
            EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
            EEPROM.commit(); //Commit EEPROM
            EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
            SerialBT.printf("SSID erased\n");
        }
        else{
            SerialBT.printf("Current SSID : %s\n", nvmSet.SSID);
            strncpy(nvmSet.SSID, ssid.c_str(), sizeof(nvmSet.SSID));
            EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
            EEPROM.commit(); //Commit EEPROM
            EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
            SerialBT.printf("New SSID : %s\n", nvmSet.SSID);
        }
    }
    else{
        SerialBT.printf("SSID : %s\n", nvmSet.SSID);
    }
}
void pass_Callback(cmd *cmdPtr){
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String pass = arg.getValue();
    if(pass.length() > 0){
        SerialBT.printf("Current PASS : %s\n", nvmSet.PASS);
        strncpy(nvmSet.PASS, pass.c_str(), sizeof(nvmSet.PASS));
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("New PASS : %s\n", nvmSet.PASS);
    }
    else{
        SerialBT.printf("PASS : %s\n", nvmSet.PASS);
    }

}
void ref_Callback(cmd *cmdPtr){
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String ref = arg.getValue();
    if(ref.length() > 0){
        SerialBT.printf("Current REF : %d\n", nvmSet.Max1161_RefVolt);
        nvmSet.Max1161_RefVolt = ref.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("New REF : %d\n", nvmSet.Max1161_RefVolt);
        _max14921.VREF = float(nvmSet.Max1161_RefVolt)/1000.0;
    }
    else{
        SerialBT.printf("REF : %d\n", nvmSet.Max1161_RefVolt);
    }
}
void again_Callback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String gain = arg.getValue();
    if(gain.length() > 0){
        SerialBT.printf("Current GAIN : %d\n", nvmSet.AmpereGain);
        nvmSet.AmpereGain = gain.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("OFFSET : %d\n", nvmSet.AmpereOffset);
        SerialBT.printf("Current [AMPERE] : %2.2f\n", _max14921.T123[AMPERE]/10.0);
        SerialBT.printf("New GAIN : %d\n", nvmSet.AmpereGain);

    }

    else{
        SerialBT.printf("GAIN : %d\n", nvmSet.AmpereGain);
        SerialBT.printf("OFFSET : %d\n", nvmSet.AmpereOffset);
        SerialBT.printf("Current [AMPERE] : %2.2f\n", _max14921.T123[AMPERE]/10.0);
    }

}
void tgain_Callback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String gain = arg.getValue();
    if(gain.length() > 0){
        SerialBT.printf("Current GAIN : %d\n", nvmSet.TotalVoltageGain);
        nvmSet.TotalVoltageGain = gain.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("OFFSET : %d\n", nvmSet.TotalVoltageOffset);
        SerialBT.printf("New GAIN : %d\n", nvmSet.TotalVoltageGain);
    }
    else{
        SerialBT.printf("GAIN : %d\n", nvmSet.TotalVoltageGain);
        SerialBT.printf("OFFSET : %d\n", nvmSet.TotalVoltageOffset);
    }
}
void vgain_Callback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String gain = arg.getValue();
    if(gain.length() > 0){
        SerialBT.printf("Current GAIN : %d\n", nvmSet.Max1161_CellGain);
        nvmSet.Max1161_CellGain = gain.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("OFFSET : %d\n", nvmSet.Max1161_CellOffset);
        SerialBT.printf("New GAIN : %d\n", nvmSet.Max1161_CellGain);
    }
    else{
        SerialBT.printf("GAIN : %d\n", nvmSet.Max1161_CellGain);
        SerialBT.printf("OFFSET : %d\n", nvmSet.Max1161_CellOffset);
    }
}

void aoffset_Callback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String offset = arg.getValue();
    if(offset.length() > 0){
        SerialBT.printf("Current OFFSET : %d\n", nvmSet.AmpereOffset);
        nvmSet.AmpereOffset = offset.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("GAIN : %d\n", nvmSet.AmpereGain);
        SerialBT.printf("Current [AMPERE] : %2.2f\n", _max14921.T123[AMPERE]/10.0);
        SerialBT.printf("New OFFSET : %d\n", nvmSet.AmpereOffset);
    }
    else{
        SerialBT.printf("OFFSET : %d\n", nvmSet.AmpereOffset);
        SerialBT.printf("GAIN : %d\n", nvmSet.AmpereGain);
        SerialBT.printf("Current [AMPERE] : %2.2f\n", _max14921.T123[AMPERE]/10.0);
    }

}
void toffset_Callback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String offset = arg.getValue();
    if (offset.length() > 0)
    {
        SerialBT.printf("Current OFFSET : %d\n", nvmSet.TotalVoltageOffset);
        nvmSet.TotalVoltageOffset = offset.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); // Write to EEPROM
        EEPROM.commit();                                             // Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));  // Read from EEPROM
        SerialBT.printf("GAIN : %d\n", nvmSet.TotalVoltageGain);
        SerialBT.printf("New OFFSET : %d\n", nvmSet.TotalVoltageOffset);
    }
    else
    {
        SerialBT.printf("OFFSET : %d\n", nvmSet.TotalVoltageOffset);
        SerialBT.printf("GAIN : %d\n", nvmSet.TotalVoltageGain);
    }
}
void voffset_Callback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String offset = arg.getValue();
    if (offset.length() > 0)
    {
        SerialBT.printf("Current OFFSET : %d\n", nvmSet.Max1161_CellOffset);
        nvmSet.Max1161_CellOffset = offset.toInt();
        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); // Write to EEPROM
        EEPROM.commit();                                             // Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet));  // Read from EEPROM
        SerialBT.printf("GAIN : %d\n", nvmSet.Max1161_CellGain);
        SerialBT.printf("New OFFSET : %d\n", nvmSet.Max1161_CellOffset);
    }
    else
    {
        SerialBT.printf("OFFSET : %d\n", nvmSet.Max1161_CellOffset);
        SerialBT.printf("GAIN : %d\n", nvmSet.Max1161_CellGain);
    }
}
void useHoleCT_Callback(cmd *cmdPtr)
{
    Command cmd(cmdPtr);
    Argument arg = cmd.getArgument(0);
    String useHoleCT = arg.getValue();
    if(useHoleCT.length() > 0){
        SerialBT.printf("Current USE HOLE CT : %d\n", nvmSet.UseHoleCT);
        //nvmSet.UseHoleCT = useHoleCT.toInt() >0  ? 1:0;
        //이 값을 CT Tatio로 변경하기 위하여 변경한다
        nvmSet.UseHoleCT = useHoleCT.toInt() ;

        EEPROM.writeBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Write to EEPROM
        EEPROM.commit(); //Commit EEPROM
        EEPROM.readBytes(0, (byte *)&nvmSet, sizeof(nvmSystemSet)); //Read from EEPROM
        SerialBT.printf("GAIN : %d\n", nvmSet.AmpereGain);
        SerialBT.printf("OFFSET : %d\n", nvmSet.AmpereOffset);
        SerialBT.printf("Current [AMPERE] : %2.2f\n", _max14921.T123[AMPERE]/10.0);
    }
    else{
        SerialBT.printf("Current USE HOLE CT : %d\n", nvmSet.UseHoleCT);
        SerialBT.printf("GAIN : %d\n", nvmSet.AmpereGain);
        SerialBT.printf("OFFSET : %d\n", nvmSet.AmpereOffset);
        SerialBT.printf("Current [AMPERE] : %2.2f\n", _max14921.T123[AMPERE]/10.0);
    }
}
void help_Callback(cmd *c){
    SerialBT.printf("%s\n", simpleCli.toString().c_str());
}
void error_Callback(cmd_error *e){
    CommandError cmdError(e); // Create wrapper object
    SerialBT.print("ERROR: ");
    SerialBT.println(cmdError.toString());
    if (cmdError.hasCommand()) {
        SerialBT.print("Did you mean \"");
        SerialBT.print(cmdError.getCommand().toString());
        SerialBT.println("\"?");
    }
}
void ip_Callback(cmd *cmdPtr){
    if(WiFi.isConnected()){
        SerialBT.printf("IPAddress: %s\n", WiFi.localIP().toString().c_str());
        SerialBT.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
        SerialBT.printf("SubnetMask: %s\n", WiFi.subnetMask().toString().c_str());
    }
    else{
        SerialBT.printf("Not connected\n");
    }
}
Command reboot;
Command cmdversion;
Command id;
Command cells;
Command ssid;
Command pass;
Command ref;
Command again;
Command aoffset;
Command vgain;
Command voffset;
Command tgain;
Command toffset;
Command useHoleCT;
Command help;
Command ip;
void simpleCliSetup()
{
    reboot = simpleCli.addCommand("reboot", reboot_Callback);
    reboot.setDescription("Reboot the device");
    cmdversion = simpleCli.addCommand("ver/sion", version_Callback);
    cmdversion.setDescription("Get the version");
    id = simpleCli.addSingleArgCmd("id", id_Callback);
    id.setDescription("Set the Modbus ID");
    cells = simpleCli.addSingleArgCmd("cells", cells_Callback);//InstalledCells
    cells.setDescription("Set the Installed Cells");
    ssid = simpleCli.addSingleArgCmd("ssid", ssid_Callback);//SSID
    ssid.setDescription("Set the SSID");
    pass = simpleCli.addSingleArgCmd("pass", pass_Callback);//PASS
    pass.setDescription("Set the PASS");
    ref = simpleCli.addSingleArgCmd("ref", ref_Callback);//REF
    ref.setDescription("Set the REF");
    again = simpleCli.addSingleArgCmd("again", again_Callback);//GAINo
    again.setDescription("Set the Ampere GAIN");
    aoffset = simpleCli.addSingleArgCmd("aoff/set", aoffset_Callback);//OFFSET
    aoffset.setDescription("Set the Ampere OFFSET");
    vgain = simpleCli.addSingleArgCmd("vgain", vgain_Callback);//GAINo
    vgain.setDescription("Set the Voltage GAIN");
    tgain = simpleCli.addSingleArgCmd("tgain", tgain_Callback);//GAINo
    tgain.setDescription("Set the Total Voltage GAIN");
    toffset = simpleCli.addSingleArgCmd("toff/set", toffset_Callback);//OFFSET
    toffset.setDescription("Set the Total Voltage OFFSET");
    voffset = simpleCli.addSingleArgCmd("voff/set", voffset_Callback);//OFFSET
    voffset.setDescription("Set the Voltage OFFSET");
    useHoleCT = simpleCli.addSingleArgCmd("use/current", useHoleCT_Callback);//USE HOLE CT
    useHoleCT.setDescription("Set the USE HOLE CT");
    ip = simpleCli.addSingleArgCmd("ip", ip_Callback);//IP
    ip.setDescription("Read the IPAddress,GW,SUBNETMASK");
    help = simpleCli.addCommand("help",help_Callback); 
    help.setDescription("Get help!");
    simpleCli.setOnError(error_Callback);
}
void bluetoothTaskSetup(void)
{
    String bleName = "sub";
    bleName += String(WiFi.macAddress());
    bleName += "_";
    bleName += String(nvmSet.ModbusAddress);
    bleName.toLowerCase();
    Serial.printf("\nBluetooth Name : %s\n", bleName.c_str());

    SerialBT.begin(bleName.c_str());
    simpleCliSetup();
}
void bluetoothTask(void *pvParameters)
{
    bluetoothTaskSetup();
    for(;;)
    {
        readInputSerialBT();
        vTaskDelay(1);
    }
}
String inputString;
void readInputSerialBT()
{
  char readBuf[2];
  char readCount = 0;
  // while (true)
  {
    if (SerialBT.available())
    {
      if (SerialBT.readBytes(readBuf, 1))
      {
        readBuf[1] = 0x00;
        if (readBuf[0] == 8)
        {
          inputString.remove(inputString.length() - 1);
        }
        else
        {
          printf("%c", readBuf[0]);
          inputString += String(readBuf);
        }
      }
      if (readBuf[0] == '\n' || readBuf[0] == '\r')
      {
        simpleCli.parse(inputString);
        while (SerialBT.available()) SerialBT.readBytes(readBuf, 1);
        inputString = "";
        SerialBT.printf("\n# ");
        // break;
      }
    }
  }
}