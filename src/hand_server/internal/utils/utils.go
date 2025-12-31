package utils

import (
	"bufio"
	"fmt"
	log "github.com/sirupsen/logrus"
	"gopkg.in/yaml.v3"
	"os"
	"path"
	"strings"
)

func AskForConfirmationDefaultYes(s string) bool {
	reader := bufio.NewReader(os.Stdin)

	fmt.Printf("%s [Y/n]: ", s)

	response, err := reader.ReadString('\n')
	if err != nil {
		log.Fatal(err)
	}

	response = strings.ToLower(strings.TrimSpace(response))

	if response == "y" || response == "yes" || response == "" {
		return true
	} else if response == "n" || response == "no" {
		return false
	} else {
		return false
	}

}

func DumpOption(opt interface{}, outputPath string, overwrite bool) {
	buffer, _ := yaml.Marshal(opt)

	parentPath := path.Dir(outputPath)
	fileInfo, err := os.Stat(parentPath)
	if os.IsNotExist(err) {
		err = os.MkdirAll(parentPath, 0700)
		if err != nil {
			log.Errorln("cannot create directory", parentPath)
			log.Exit(1)
		}
	}

	fileInfo, err = os.Stat(parentPath)
	if os.IsPermission(err) || fileInfo.Mode() != 0700 {
		err = os.Chmod(parentPath, 0700)
		if err != nil {
			log.Errorln("cannot read director", parentPath)
			log.Exit(1)
		}
	}

	if !overwrite {
		if _, err := os.Stat(outputPath); !os.IsNotExist(err) {
			ret := AskForConfirmationDefaultYes("configuration " + outputPath + " already exist, overwrite?")
			if !ret {
				log.Infoln("abort")
				return
			}
		}
	}

	log.Infoln("writing default configuration to", outputPath)
	f, err := os.OpenFile(outputPath, os.O_CREATE|os.O_RDWR|os.O_TRUNC, 0600)
	defer func() { _ = f.Close() }()
	if err != nil {
		panic("cannot open " + outputPath + ", check permissions")
	}

	w := bufio.NewWriter(f)
	_, err = w.Write(buffer)
	if err != nil {
		log.Panicln("cannot write configuration", err)
	}
	_ = w.Flush()
	_ = f.Close()

}
