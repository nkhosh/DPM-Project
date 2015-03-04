import java.io.DataInputStream;
import java.io.DataOutputStream;

import lejos.nxt.comm.NXTCommConnector;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.RS485;


public class NXTSlave {
	Robot007 robot007;
	public NXTSlave(Robot007 robot) {
		this.robot007 = robot;
	}
	public static void main(String[] args) throws Exception {
		NXTCommConnector connector = RS485.getConnector();
		int mode = NXTConnection.PACKET;
		
		while(true) {
			NXTConnection con = connector.waitForConnection(0, mode);
			
			DataInputStream input = con.openDataInputStream();
			DataOutputStream output = con.openDataOutputStream();
			
			while(true) {
				int n;
				try {
					n = input.readInt();
				}
				catch (Exception e) {
					break;
				}
				output.writeInt(0);
				output.flush();
			}
			input.close();
			output.close();
			con.close();
		}
		
	}
}
