package frc.robot.config;

import java.io.File;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;

import com.fasterxml.jackson.databind.ObjectMapper;

public class ConstantsFactory {
    private String filename;

    public ConstantsFactory(String filename) {
        this.filename = filename;
    }

    public Constants getConstants(Class aClass) {
        ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
        mapper.findAndRegisterModules();

        try {
            Constants newConstants = (Constants) mapper.readValue(new File(filename), aClass);
            newConstants.setFactory(this);
            return newConstants;
        } catch (Exception e) {
            return null;
        }
    }

    public void saveConstants(Object anObject) {
        ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
        mapper.findAndRegisterModules();

        try {
            mapper.writeValue(new File(filename), anObject);
        } catch (Exception e) {

        }
    }
}
