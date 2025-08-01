// Simple test to check IPC communication directly
use std::time::Duration;
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Testing direct IPC communication...");
    
    // Test if we can connect to the IPC server
    match tokio::net::TcpStream::connect("127.0.0.1:8765").await {
        Ok(_stream) => {
            println!("✅ Successfully connected to IPC server on port 8765");
        }
        Err(e) => {
            println!("❌ Failed to connect to IPC server: {}", e);
            return Ok(());
        }
    }
    
    println!("IPC server is reachable!");
    Ok(())
}