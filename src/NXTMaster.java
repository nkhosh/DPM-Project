import java.io.DataInputStream;
import java.io.DataOutputStream;

import lejos.nxt.comm.NXTCommConnector;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.RS485;


public class NXTMaster {
	Robot007 robot007;
	public NXTMaster(Robot007 robot) {
		this.robot007 = robot;
	}
	public static void main(String[] args) throws Exception {
		NXTCommConnector connector = RS485.getConnector();
		int mode = NXTConnection.PACKET;
		String name = "NXT";
		NXTConnection con = connector.connect(name, mode);
		
		if (con == null)
        {
            Thread.sleep(2000);
            System.exit(1);
        }
		
		DataInputStream input = con.openDataInputStream();
		DataOutputStream output = con.openDataOutputStream();
		
		
	}
}
