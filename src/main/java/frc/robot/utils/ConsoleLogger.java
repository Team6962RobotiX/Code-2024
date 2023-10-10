package frc.robot.utils;

import java.io.PrintStream;
import java.lang.Thread.UncaughtExceptionHandler;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class ConsoleLogger extends PrintStream {
  private static StringLogEntry consoleOutput = new StringLogEntry(DataLogManager.getLog(), "consoleOutput");
  
  public ConsoleLogger(PrintStream stream) {
    super(stream);
    System.setOut(this);
    System.setErr(this);
  }

  @Override
  public void print(String s) {
    consoleOutput.append(s);
    super.print(s);
  }

  @Override
  public void println(String s) {
    consoleOutput.append(s);
    super.println(s);
  }
}