package frc.robot.config;

public class Constants {
    private ConstantsFactory myFactory;
    private String name;

    public void setFactory(ConstantsFactory aFactory) {
        myFactory = aFactory;
    }

    public String getName() {
        return name;
    }

    public void setName(String aName) {
        this.name = aName;
    }

    public void saveConstants() {
        myFactory.saveConstants((Constants) this);
    }
}
