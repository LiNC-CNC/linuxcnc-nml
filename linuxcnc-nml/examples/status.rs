use linuxcnc_nml::NmlClient;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = NmlClient::open_default()?;
    let status = client.poll_status()?;
    println!("linuxcnc {}", NmlClient::version());
    println!("valid: {}", status.valid);
    println!("echo serial: {}", status.echo_serial_number);
    println!("task: {:?}/{:?}", status.task.mode, status.task.state);
    println!("interp: {:?}", status.task.interp_state);
    println!("file: {}", status.task.file);
    println!(
        "actual: X {:.4} Y {:.4} Z {:.4}",
        status.positions.actual.x, status.positions.actual.y, status.positions.actual.z
    );
    println!(
        "g5x {}: X {:.4} Y {:.4} Z {:.4}",
        status.offsets.g5x_index,
        status.offsets.g5x_offset.x,
        status.offsets.g5x_offset.y,
        status.offsets.g5x_offset.z
    );
    println!(
        "joints: {} active, {} slots",
        status.motion.joint_count,
        status.joints.len()
    );
    for (index, joint) in status
        .joints
        .iter()
        .take(status.motion.joint_count)
        .enumerate()
    {
        println!(
            "joint {index}: input {:.4} homed {} ferror {:.6}",
            joint.input, joint.homed, joint.ferror_current
        );
    }
    Ok(())
}
