package org.myrobotlab.service.config;

/**
 *  Email config gets turned into javax email props
 */
public class EmailConfig extends ServiceConfig {

  public String to; // if set sends auto
  public String format = "text/html"; // text/html or text/plain
  public String user = null;
  public String host = null;
  public int port = 25; /* 465, 587 */
  public String from = null;
  boolean auth = true;
  boolean starttls = true;
  boolean debug = true;
  boolean starttlsRequired = true;
  String protocols = "TLSv1.2";
  String socketFactory = "javax.net.ssl.SSLSocketFactory";

  public String pass = null;

}
