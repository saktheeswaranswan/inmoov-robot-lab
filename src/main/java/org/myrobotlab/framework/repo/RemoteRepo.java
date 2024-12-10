package org.myrobotlab.framework.repo;

public class RemoteRepo {

  public String id;
  // public String name;
  public String url;
  public String comment;
  public boolean snapshots = false;

  public RemoteRepo(String id, String url, String comment) {
    this.id = id;
    this.url = url;
    this.comment = comment;
  }

  public RemoteRepo(String id, String url) {
    this(id, url, null);
  }

  public RemoteRepo(String id, String url, boolean snapshots) {
    this(id, url, null);
    this.snapshots = snapshots;
  }

  public RemoteRepo(String id, String url, String comment, boolean snapshots) {
    this(id, url, comment);
    this.snapshots = snapshots;
  }

}
