package nad_nav

import (
	"fmt"
	"net"
)

// OutputSender sends controller commands over UDP as CSV.
type OutputSender struct {
	conn *net.UDPConn
}

// NewOutputSender creates a UDP sender for the given address.
func NewOutputSender(addr string) (*OutputSender, error) {
	if addr == "" {
		return &OutputSender{}, nil
	}
	udpAddr, err := net.ResolveUDPAddr("udp", addr)
	if err != nil {
		return nil, err
	}
	conn, err := net.DialUDP("udp", nil, udpAddr)
	if err != nil {
		return nil, err
	}
	return &OutputSender{conn: conn}, nil
}

// Close releases the UDP socket.
func (s *OutputSender) Close() error {
	if s == nil || s.conn == nil {
		return nil
	}
	return s.conn.Close()
}

// Send writes "yaw,vertical,forward,mode" as a CSV payload.
func (s *OutputSender) Send(cmd BodyCommand) {
	if s == nil || s.conn == nil {
		return
	}
	payload := fmt.Sprintf("%.4f,%.4f,%.4f,%s", cmd.Yaw, cmd.Vertical, cmd.Forward, cmd.Mode.String())
	_, _ = s.conn.Write([]byte(payload))
}
