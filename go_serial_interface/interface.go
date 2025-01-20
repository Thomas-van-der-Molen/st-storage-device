package main

import (
	"fmt"
	"go.bug.st/serial"
)

func main() {
	//fmt.Println("Hello, windows");
	//wd, _ := os.Getwd();
	//fmt.Println("cwd: ", wd);
	// ports, err := serial.GetPortsList()
	// if err != nil {
	// 	fmt.Println("error");
	// }
	// if len(ports) == 0 {
	// 	fmt.Println("no ports found");
	// }
	// for _, port := range ports {
	// 	fmt.Printf("Found port: %v\n", port)
	// }

	mode := &serial.Mode{
		BaudRate: 115200,
		Parity: serial.NoParity,
		DataBits: 8,
		StopBits: 1,
	}

	port, err := serial.Open("COM6", mode)
	if err != nil {
		fmt.Println("error");
	}

	n, _ :=port.Write([]byte("ten chars\n"));
	fmt.Printf("sent %d bytes\n", n);

	rx_buf := make([]byte, 100);
	for {

		n, _ := port.Read(rx_buf);
		fmt.Printf("%v", string(rx_buf[:n]));
	}

}

