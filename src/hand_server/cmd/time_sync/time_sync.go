package main

import (
	"context"
	"fmt"
	"github.com/gin-gonic/gin"
	"github.com/gogf/gf/v2/os/gproc"
	log "github.com/sirupsen/logrus"
	"github.com/spf13/cobra"
	"net/http"
	"os"
	"regexp"
	"strconv"
	"sync"
	"time"
)

var globalErrCnt = 1
var globalSyncSuccess = true
var globalSyncSuccessTS = time.Date(0, 0, 0, 0, 0, 0, 0, time.UTC)

const maxSyncRetry = 3
const syncRetryIntervalS = 5
const syncReTriggerIntervalS = 300
const ntpdateCmd = "ntpdate"

type NTPDConf struct {
	Server string `json:"server"`
}

var lock = &sync.Mutex{}

func execNTPUpdate(hostname string) (string, error) {
	lock.Lock()
	defer lock.Unlock()
	output, err := gproc.ShellExec(context.Background(), ntpdateCmd+" "+hostname)
	return output, err
}

func setup(serverAddress string) {
	for range [maxSyncRetry]struct{}{} {
		output, err := execNTPUpdate(serverAddress)
		if err == nil {
			globalSyncSuccess = true
			globalSyncSuccessTS = time.Now()
			return
		}
		log.Info("The " + strconv.Itoa(globalErrCnt) + "th sync trial to " + serverAddress + " failed: " + output + ". Wait 5 seconds")
		time.Sleep(syncRetryIntervalS * time.Second)
		globalErrCnt++
	}
	globalSyncSuccess = false
	globalSyncSuccessTS = time.Date(0, 0, 0, 0, 0, 0, 0, time.UTC)
}

func readNTPServerFromConfig(ntpdConfPath string) (string, error) {
	bytes, err := os.ReadFile(ntpdConfPath)
	if err != nil {
		return "", err
	}
	re := regexp.MustCompile(`# MANAGED BY RFMOCAP\r?\nserver\s([^\s]*)\siburst\r?\n# MANAGED BY RFMOCAP`)
	matches := re.FindStringSubmatch(string(bytes))

	if len(matches) < 2 {
		return "", fmt.Errorf("No server address found in ntp.conf")
	}
	server := matches[1]
	return server, nil
}

func writeNTPServerToConfig(ntpdConfPath string, server string) error {
	bytes, err := os.ReadFile(ntpdConfPath)
	if err != nil {
		return err
	}
	re := regexp.MustCompile(`# MANAGED BY RFMOCAP\r?\nserver\s([^\s]*)\siburst\r?\n# MANAGED BY RFMOCAP`)
	newStr := ""
	if re.MatchString(string(bytes)) {
		newStr += re.ReplaceAllString(string(bytes), "# MANAGED BY RFMOCAP\nserver "+server+" iburst\n# MANAGED BY RFMOCAP")
	} else {
		newStr += string(bytes) + "\n# MANAGED BY RFMOCAP\nserver " + server + " iburst\n# MANAGED BY RFMOCAP"
	}
	err = os.WriteFile(ntpdConfPath, []byte(newStr), 0644)
	if err != nil {
		return err
	}
	return nil
}

func _main(cmd *cobra.Command) (err error) {
	// Parse the flags
	serverHostname, err := cmd.Flags().GetString("server")
	listenPort, err := cmd.Flags().GetInt("port")
	ntpdConfPath, err := cmd.Flags().GetString("ntpd_config_path")
	log.Info("NTP server:", serverHostname)
	log.Info("NTPD config path:", ntpdConfPath)

	go func() {
		for {
			setup(serverHostname)
			time.Sleep(syncReTriggerIntervalS * time.Minute)
		}
	}()

	router := gin.Default()

	router.GET("/time-sync", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"globalErrCnt":        globalErrCnt,
			"globalSyncSuccess":   globalSyncSuccess,
			"globalSyncSuccessTS": globalSyncSuccessTS.Unix(),
		})
	})

	router.GET("/ntpd-config", func(c *gin.Context) {
		addr, err := readNTPServerFromConfig(ntpdConfPath)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{
				"err":  err.Error(),
				"addr": nil,
			})
			return
		} else {
			c.JSON(http.StatusOK, gin.H{
				"err":  nil,
				"addr": addr,
			})
		}
	})

	router.POST("/time-sync", func(c *gin.Context) {
		output, err := execNTPUpdate(serverHostname)
		if err != nil {
			globalErrCnt++
			c.JSON(http.StatusInternalServerError, gin.H{
				"err": err.Error(),
				"msg": "Synchronization failed: " + output,
			})
		} else {
			globalSyncSuccessTS = time.Now()
			globalSyncSuccess = true
			c.JSON(http.StatusOK, gin.H{
				"err": nil,
				"msg": "Synchronization success",
			})
		}
	})

	router.PUT("/ntpd-config", func(c *gin.Context) {
		req := NTPDConf{}
		err = c.BindJSON(&req)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{
				"err": err.Error(),
			})
			return
		}
		serverHostname = req.Server
		err := writeNTPServerToConfig(ntpdConfPath, serverHostname)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{
				"err": err.Error(),
			})
		} else {
			_, err := gproc.ShellExec(context.Background(), "systemctl restart ntpd")
			if err != nil {
				c.JSON(http.StatusInternalServerError, gin.H{
					"err": err.Error(),
					"msg": "The NTP server address now is set as: " + serverHostname,
				})
				return
			} else {
				c.JSON(http.StatusOK, gin.H{
					"err": nil,
					"msg": "The NTP server address now is set as: " + serverHostname,
				})
			}
		}
	})

	err = router.Run(":" + strconv.Itoa(listenPort))

	return err
}

var rootCmd = &cobra.Command{
	Use:   "ntp_synchronizer",
	Short: "ntp synchronizer between client and server",
	Long:  "This programs tries to maintain a ntp synchronizer as a deamon service",
	Run: func(cmd *cobra.Command, args []string) {
		if err := _main(cmd); err != nil {
			fmt.Println(err)
			os.Exit(1)
		}
	},
}

func main() {
	rootCmd.Flags().String("server", "pool.ntp.org", "NTP server's Hostname")
	rootCmd.Flags().Int("port", 8080, "Port number")
	rootCmd.Flags().String("ntpd_config_path", "/etc/ntpsec/ntp.conf", "NTPD's config path")
	rootCmd.Flags().Bool("debug", false, "toggle debug logging")
	err := rootCmd.Execute()
	if err != nil {
		log.Fatal(err)
	}
}
